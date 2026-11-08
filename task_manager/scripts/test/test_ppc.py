#!/usr/bin/env python3
"""PPC behaviour tree against a fake world (no robot, no running ROS graph).
python3 -m pytest task_manager/scripts/test/test_ppc.py"""

import importlib.util
import json
import os
from dataclasses import asdict
from types import SimpleNamespace

import pytest

from task_manager.subtask_managers.composite_tasks import CompositeTasks
from task_manager.utils.bt import RunLog, TreeRunner
from task_manager.utils.status import Status

OK, ERR = Status.EXECUTION_SUCCESS, Status.EXECUTION_ERROR
SHELVES = {1: 0.095, 2: 0.39, 3: 0.68}


def _load_tm():
    path = os.path.join(os.path.dirname(__file__), "..", "pickandplace_task_manager.py")
    spec = importlib.util.spec_from_file_location("ppc_tm", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


tm = _load_tm()


def det(name, z=0.8):
    return SimpleNamespace(classname=name, px=0.5, py=0.0, pz=z)


class World:
    """Objects per furniture; the fake areas read and change it."""

    def __init__(self, cost_s=1.0):
        self.t = 0.0
        self.cost_s = cost_s
        self.loc = "safe_place"
        self.items = {
            "dinner_table": [det("pringles"), det("spoon"), det("apple"), det("red_plate")],
            "dishwasher": [det("bowl"), det("spoon")],
            "cabinet": [det("cornflakes", 0.40), det("milk", 0.70)],
        }
        self.held = None
        self.calls = []

    def tick(self, call):
        self.calls.append(call)
        self.t += self.cost_s

    def here(self):
        return self.items.setdefault(self.loc, [])


class FakeVision:
    def __init__(self, w):
        self.w = w

    def detect_objects(self, timeout=5):
        return OK, list(self.w.here())

    def detect_chairs_to_remove(self):
        return OK, []

    def moondream_query(self, prompt):
        return OK, "yes"


class FakeNav:
    def __init__(self, w):
        self.w = w
        self.fail = set()

    def move_to_location(self, room, place):
        self.w.tick(f"nav:{place}")
        if place in self.fail:
            return ERR, "blocked"
        self.w.loc = place
        return OK, ""

    def dock_table(self, offset=0.0):
        return OK, ""

    def check_door(self):
        return OK, ""


class FakeManip:
    def __init__(self, w):
        self.w = w
        self.pick_failures = {}  # name -> failures left (-1 = always)
        self.bad_shelves = set()

    def move_to_position(self, *a, **k):
        return OK

    def get_optimal_position_for_plane(self, *a, **k):
        return OK

    def pick_object(self, name, in_configuration=False, scan_environment=False):
        self.w.tick(f"pick:{name}")
        left = self.pick_failures.get(name, 0)
        if left:
            self.pick_failures[name] = left - 1 if left > 0 else -1
            return ERR
        match = next((d for d in self.w.here() if d.classname == name), None)
        if match is None:
            return ERR
        self.w.here().remove(match)
        self.w.held = match
        return OK

    def _drop(self, z=0.8):
        if self.w.held is not None:
            self.w.here().append(det(self.w.held.classname, z))
            self.w.held = None

    def place(self, close_to="", is_trash=False, **k):
        self.w.tick("place")
        self._drop()
        return OK

    def place_on_shelf(self, plane_height, tolerance):
        self.w.tick(f"shelf:{plane_height}")
        if plane_height in self.bad_shelves:
            return ERR
        self._drop(plane_height + 0.05)
        return OK

    def pour(self, pour_object_name, pour_container_name, object_already_grasped=False):
        self.w.tick(f"pour:{pour_object_name}")
        return OK


class FakeHri:
    mock_data = False
    start_button_clicked = True

    def __init__(self, w):
        self.w = w

    def say(self, text, wait=True):
        return OK

    def publish_display_topic(self, *a, **k):
        pass

    def publish_display_capture(self, *a, **k):
        pass

    def publish_display_step(self, *a, **k):
        pass

    def confirm(self, *a, **k):
        return OK, "yes"

    def categorize_objects(self, table_objects, shelves):
        return OK, {1: ["snacks"]}, {1: list(table_objects)}, {}


class FakeTF:
    def transform(self, stamped, frame, timeout=None):
        return stamped


class FakeNode:
    def create_subscription(self, *a, **k):
        pass

    def create_client(self, *a, **k):
        return SimpleNamespace(wait_for_service=lambda timeout_sec: False)

    def get_logger(self):
        return SimpleNamespace(info=lambda m: None, warn=lambda m: None, error=lambda m: None)


def make(cost_s=1.0, state=None, **cfg):
    w = World(cost_s)
    node = FakeNode()
    sm = SimpleNamespace(
        vision=FakeVision(w), nav=FakeNav(w), manipulation=FakeManip(w), hri=FakeHri(w)
    )
    sm.composite = CompositeTasks(node, sm.vision, sm.nav, sm.manipulation, sm.hri, FakeTF())
    sm.composite.wait = lambda s: None
    config = tm.PPCConfig(yolo_names={"cereal": "cornflakes", "milk": "milk"}, **cfg)
    ppc = tm.PickAndPlace(sm, config, state or tm.PPCState(), node, dict(SHELVES), lambda: w.t)
    ppc.object_to_category = {"apple": "fruit"}
    return ppc, w, sm


def run(ppc, tmp_path, start_at="perceive"):
    if ppc.state.started_at is None:
        ppc.state.started_at = ppc.clock()
    log = RunLog(str(tmp_path), lambda: asdict(ppc.state))
    return TreeRunner(ppc.build_tree(start_at), log, ppc.past_hard_deadline).run(), log


def status_of(ppc, name):
    return next(o.status for o in ppc.state.objects if o.name == name)


def test_happy_path(tmp_path):
    ppc, w, _ = make()
    result, log = run(ppc, tmp_path)
    assert result.name == "SUCCESS"
    assert [o.status for o in ppc.state.objects].count("placed") == 3
    assert status_of(ppc, "red_plate") == "skipped"
    assert ppc.state.objects[0].name == "pringles"  # "other" goes first
    assert [b.status for b in ppc.state.breakfast] == ["placed"] * 4
    assert "pour:cornflakes" in w.calls and "pour:milk" in w.calls
    assert w.held is None and ppc.state.holding is None
    assert (log.dir / "run_log.jsonl").exists()


def test_failed_pick_skips_object(tmp_path):
    ppc, w, sm = make()
    sm.manipulation.pick_failures["pringles"] = -1
    run(ppc, tmp_path)
    assert status_of(ppc, "pringles") == "skipped"
    assert w.calls.count("pick:pringles") == ppc.cfg.attempts
    assert status_of(ppc, "spoon") == "placed" and status_of(ppc, "apple") == "placed"


def test_pick_retry_succeeds(tmp_path):
    ppc, w, sm = make()
    sm.manipulation.pick_failures["pringles"] = 1
    run(ppc, tmp_path)
    assert status_of(ppc, "pringles") == "placed"


def test_cabinet_falls_back_to_next_shelf(tmp_path):
    ppc, w, sm = make()
    sm.manipulation.bad_shelves = {0.39}
    run(ppc, tmp_path)
    assert status_of(ppc, "pringles") == "placed"
    assert "shelf:0.68" in w.calls


def test_trash_and_cutlery_destinations(tmp_path):
    ppc, w, _ = make(max_cleanup_objects=3)
    run(ppc, tmp_path)
    names = lambda loc: [d.classname for d in w.items.get(loc, [])]  # noqa: E731
    assert "apple" in names("trashbin")
    assert "spoon" in names("cooking_table")


def test_resume_while_holding_places_it_first(tmp_path):
    state = tm.PPCState(holding="apple")
    ppc, w, _ = make(state=state)
    w.held = det("apple")
    run(ppc, tmp_path, start_at="breakfast")
    assert w.calls.index("place") < w.calls.index("pick:bowl")
    assert ppc.state.breakfast[0].status == "placed"


def test_cleanup_deadline_moves_to_breakfast(tmp_path):
    ppc, w, _ = make(cost_s=30.0, cleanup_deadline_s=60.0)
    run(ppc, tmp_path)
    assert [o.status for o in ppc.state.objects].count("placed") < 3
    assert "pick:bowl" in w.calls


def test_breakfast_tail_drops_non_essential(tmp_path):
    ppc, w, _ = make(cost_s=30.0, cleanup_deadline_s=0.0, breakfast_tail_s=60.0)
    run(ppc, tmp_path)
    spoon = next(b for b in ppc.state.breakfast if b.name == "spoon")
    assert spoon.status == "pending" and "pick:spoon" not in w.calls
    assert "pick:cornflakes" in w.calls


def test_hard_deadline_stops(tmp_path):
    ppc, w, _ = make(cost_s=30.0, hard_deadline_s=50.0)
    result, _ = run(ppc, tmp_path)
    assert result.name == "INVALID"


def test_state_roundtrip(tmp_path):
    ppc, _, _ = make()
    run(ppc, tmp_path)
    data = json.loads(json.dumps(asdict(ppc.state)))
    restored = asdict(tm.PPCState.from_dict(data))
    assert json.dumps(restored, sort_keys=True) == json.dumps(asdict(ppc.state), sort_keys=True)


def test_bad_start_at():
    ppc, _, _ = make()
    with pytest.raises(ValueError):
        ppc.build_tree("nope")
