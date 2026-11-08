#!/usr/bin/env python3

"""
Pick and Place Challenge - RoboCup @Home 2026, as a behaviour tree.

Cleanup (up to max_cleanup_objects, until cleanup_deadline_s) then breakfast
(bowl, cereal, milk, spoon with pours), all inside the 420 s window.
Debug: -p start_at:=perceive|cleanup|breakfast -p state_file:=<snapshot.json>
-p mock_areas:=navigation,vision -p step:=true
"""

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from frida_constants.vision_constants import (
    CHAIR_REMOVAL_IMAGE_TOPIC,
    DETECTIONS_IMAGE_TOPIC,
    IMAGE_ORIENTED_TOPIC,
)
from task_manager.utils.bt import Action, ForEach, RunLog, TreeRunner, optional, sequence
from task_manager.utils.colored_logger import CLog
from task_manager.utils.status import Status
from task_manager.utils.task import Task


@dataclass(frozen=True)
class BreakfastSpec:
    name: str  # vision label
    source: str  # map location
    close_to: str
    essential: bool  # still attempted after breakfast_tail_s
    pour: bool = False


@dataclass(frozen=True)
class PPCConfig:
    """Competition config — adjust before each run."""

    trash_category: str = "fruit"
    trash_exceptions: tuple = ()
    use_dishwasher: bool = False  # ask to open it and place inside
    place_dishwasher_at_tab: bool = (
        True  # no inside-place primitive yet: top place at cooking_table
    )
    use_extra_surface: bool = False  # clean the counter's common objects instead (-20 pts/obj)
    use_grasp_detector: bool = False
    use_vision_confirmation: bool = True
    use_octomap_clearing: bool = False  # off: a cleared octomap lets plans cross the shelf
    max_cleanup_objects: int = 3
    attempts: int = 2
    pour_attempts: int = 2
    cleanup_deadline_s: float = 180.0
    breakfast_tail_s: float = 360.0
    hard_deadline_s: float = 405.0
    skip_names: tuple = ("red_plate", "plate", "dish")  # flat objects are not graspable
    cutlery: tuple = ("fork", "knife", "spoon")
    tableware: tuple = ("red_plate", "cup", "mug", "bowl")
    # Reliability first: a box/can is the safest first pick (+100 first-pick bonus)
    priority: tuple = ("other", "cutlery", "tableware", "trash", "common")
    breakfast: tuple = (
        BreakfastSpec("bowl", "dishwasher", "", True),
        BreakfastSpec("cornflakes", "cabinet", "bowl", True, pour=True),
        BreakfastSpec("milk", "cabinet", "cornflakes", True, pour=True),
        BreakfastSpec("spoon", "dishwasher", "bowl", False),
    )


@dataclass
class TableObject:
    name: str
    category: str
    point: tuple | None = None  # base_link xyz at perceive time
    status: str = "pending"  # pending | picked | placed | skipped


@dataclass
class BreakfastItem:
    name: str
    status: str = "pending"


@dataclass
class PPCState:
    """Everything the task remembers; dumped after every leaf and loadable with state_file."""

    started_at: float | None = None
    location: str | None = None
    holding: str | None = None
    objects: list = field(default_factory=list)
    breakfast: list = field(default_factory=list)
    shelves: dict = field(default_factory=dict)
    object_to_shelf: dict = field(default_factory=dict)
    shelf_scanned: bool = False
    cabinet_scan_fresh: bool = False
    dishwasher_open: bool = False
    first_pick_done: bool = False

    @classmethod
    def from_dict(cls, d: dict) -> "PPCState":
        d = dict(d)
        d["objects"] = [TableObject(**o) for o in d.get("objects", [])]
        d["breakfast"] = [BreakfastItem(**b) for b in d.get("breakfast", [])]
        d["shelves"] = {int(k): v for k, v in d.get("shelves", {}).items()}
        return cls(**d)


# ── Task rules (no ROS) ──


def categorize(name: str, cfg: PPCConfig, object_to_category: dict) -> str:
    name = name.lower()
    if name in cfg.cutlery:
        return "cutlery"
    if name in cfg.tableware:
        return "tableware"
    trash = cfg.trash_category.lower()
    if trash and name not in cfg.trash_exceptions:
        if name == trash or object_to_category.get(name, "").lower() == trash:
            return "trash"
    return "other"


def destination(category: str, cfg: PPCConfig) -> str:
    if category == "trash":
        return "trashbin"
    if category in ("cutlery", "tableware"):
        return "cooking_table" if cfg.place_dishwasher_at_tab else "dishwasher"
    return "cabinet"


def sort_for_pick(objects: list, priority: tuple) -> list:
    """Most reliable category first."""
    # TODO(vision): report stacked objects so the ones on top are picked first
    rank = {c: i for i, c in enumerate(priority)}
    return sorted(objects, key=lambda o: rank.get(o.category, 99))


class PickAndPlace:
    """Task logic and tree; ROS-free so tests can drive it with fake areas."""

    def __init__(self, sm, cfg: PPCConfig, state: PPCState, node, clock=time.time):
        self.sm = sm
        self.cfg = cfg
        self.state = state
        self.node = node
        self.clock = clock
        self.shelf_heights = sm.composite.shelf_heights()
        if not state.breakfast:
            state.breakfast = [BreakfastItem(b.name) for b in cfg.breakfast]
        self.object_to_category = self._load_categories()

    @staticmethod
    def _load_categories() -> dict:
        try:
            path = Path(get_package_share_directory("frida_constants")) / "data" / "objects.json"
            return json.loads(path.read_text()).get("object_to_category", {})
        except Exception:
            return {}

    def elapsed(self) -> float:
        return 0.0 if self.state.started_at is None else self.clock() - self.state.started_at

    def say(self, text: str, wait: bool = False) -> Status:
        """Speak; a TTS failure never fails the task."""
        self.sm.hri.say(text, wait=wait)
        return Status.EXECUTION_SUCCESS

    def log(self, msg: str, level: str = "info"):
        CLog.fsm(self.node, "PPC", msg, level=level)

    @property
    def table(self) -> str:
        return "counter" if self.cfg.use_extra_surface else "dinner_table"

    def spec(self, name: str) -> BreakfastSpec:
        return next(b for b in self.cfg.breakfast if b.name == name)

    # ── Leaves ──

    def wait_start(self) -> Status:
        self.say("Waiting for start button to be pressed.")
        while not (self.sm.hri.start_button_clicked or self.sm.hri.mock_data):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.say("Waiting for the door to open.")
        while self.sm.nav.check_door()[0] != Status.EXECUTION_SUCCESS:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.state.started_at = self.clock()  # the scored window starts at door open
        return Status.EXECUTION_SUCCESS

    def request_chair_removal(self) -> Status:
        self.say("I am ready.")
        self.go("safe_place", say=False, dock=False)
        status, chairs = self.sm.vision.detect_chairs_to_remove()
        if status == Status.EXECUTION_SUCCESS and not chairs:
            return Status.EXECUTION_SUCCESS
        if status == Status.EXECUTION_SUCCESS:
            n = len(chairs)
            self.sm.hri.publish_display_topic(CHAIR_REMOVAL_IMAGE_TOPIC)
            self.say(f"Please remove the {n} chair{'s' if n > 1 else ''} shown on my screen.", True)
        else:
            self.say("Please remove the chairs from the dining table.", wait=True)
        self.sm.composite.wait(5.0)
        self.sm.hri.publish_display_topic(IMAGE_ORIENTED_TOPIC)
        return Status.EXECUTION_SUCCESS

    def go(self, location: str, say: bool = True, dock: bool = True) -> Status:
        """Navigate + dock once per visit; staying put is free."""
        if self.state.location == location:
            return Status.EXECUTION_SUCCESS
        self.state.cabinet_scan_fresh = False
        status = self.sm.composite.navigate_and_dock(location, dock=dock, say=say)
        self.state.location = location if status == Status.EXECUTION_SUCCESS else None
        return status

    def perceive_table(self) -> Status:
        self.go(self.table, say=False)
        self.sm.manipulation.move_to_position("table_stare")
        self.sm.hri.publish_display_topic(DETECTIONS_IMAGE_TOPIC)
        status, dets = self.sm.vision.detect_objects(timeout=5)
        if status != Status.EXECUTION_SUCCESS or not dets:
            self.log("No objects on the table, going to breakfast.", "warn")
            return Status.EXECUTION_ERROR
        self.state.objects = []
        for det in dets:
            name = (det.classname or "unknown").split("/", 1)[-1]  # "trash/<name>" from vision
            category = (
                "common"
                if self.cfg.use_extra_surface
                else categorize(name, self.cfg, self.object_to_category)
            )
            self.state.objects.append(
                TableObject(name, category, self.sm.composite.detection_point(det))
            )
        self.say(f"I have detected {len(dets)} objects on the table.", wait=True)
        self.sm.composite.announce_objects([o.name for o in self.state.objects])
        return Status.EXECUTION_SUCCESS

    def sort_objects(self) -> Status:
        for obj in self.state.objects:
            if obj.name.lower() in self.cfg.skip_names:
                obj.status = "skipped"
        self.state.objects = sort_for_pick(self.state.objects, self.cfg.priority)
        self.log(f"Pick order: {[o.name for o in self.state.objects if o.status == 'pending']}")
        return Status.EXECUTION_SUCCESS

    def ensure_gripper_empty(self) -> Status:
        status = self.sm.composite.ensure_gripper_empty(
            self.state.holding, self.cfg.attempts, self.cfg.use_grasp_detector
        )
        if status == Status.EXECUTION_SUCCESS:
            self.state.holding = None
        return status

    def _picked(self, name: str):
        self.state.holding = name
        if not self.state.first_pick_done:
            self.state.first_pick_done = True
            self.log("First pick bonus achieved.", "success")
        self.sm.hri.publish_display_capture(f"Picked {name}")

    def pick_object(self, obj: TableObject) -> Status:
        self.say(f"I will pick the {obj.name}.")
        status = self.sm.composite.pick_confirmed(
            obj.name,
            self.cfg.attempts,
            self.cfg.use_vision_confirmation,
            self.cfg.use_grasp_detector,
        )
        if status == Status.EXECUTION_SUCCESS:
            obj.status = "picked"
            self._picked(obj.name)
        return status

    def check_dishwasher(self) -> Status:
        status, answer = self.sm.vision.moondream_query(
            "Is the dishwasher door open? Reply only with yes or no."
        )
        if status != Status.EXECUTION_SUCCESS or "yes" not in answer.lower():
            self.sm.composite.ask_human(
                "Could you please open the dishwasher door for me?",
                "Please say yes once the dishwasher door is open.",
            )
        self.state.dishwasher_open = True  # assume open after asking
        return Status.EXECUTION_SUCCESS

    def scan_cabinet_shelves(self) -> Status:
        if self.state.shelf_scanned:
            return Status.EXECUTION_SUCCESS
        self.say("Scanning cabinet shelves.")
        self.state.shelves = self.sm.composite.scan_shelves(self.shelf_heights)
        self.state.shelf_scanned = True
        self.state.cabinet_scan_fresh = True
        others = [o.name for o in self.state.objects if o.category == "other"]
        try:
            status, shelf_cats, to_add, _ = self.sm.hri.categorize_objects(
                others, self.state.shelves
            )
        except Exception as e:
            self.log(f"Categorization raised {e!r}", "error")
            status = Status.EXECUTION_ERROR
        if status != Status.EXECUTION_SUCCESS:
            self.log("Categorization failed, using default shelves.", "warn")
            return Status.EXECUTION_SUCCESS
        for idx, names in to_add.items():
            for name in names:
                self.state.object_to_shelf.setdefault(name, int(idx))
        for name, idx in self.state.object_to_shelf.items():
            cats = " and ".join(shelf_cats.get(idx, []))
            self.say(
                f"The {name} goes on shelf {idx + 1}" + (f", the {cats} shelf." if cats else ".")
            )
        return Status.EXECUTION_SUCCESS

    def _shelf_heights_for(self, name: str) -> list[float]:
        """Categorized shelf first, then the rest lowest first."""
        idx = self.state.object_to_shelf.get(name)
        if idx is None or idx >= len(self.shelf_heights):
            return list(self.shelf_heights)
        primary = self.shelf_heights[idx]
        return [primary] + [h for h in self.shelf_heights if h != primary]

    def place_object(self, obj: TableObject, dest: str) -> Status:
        self.say(f"Placing the {obj.name} at the {dest.replace('_', ' ')}.")
        if dest == "trashbin":
            status = self.sm.manipulation.place(is_trash=True)
        elif dest == "cabinet":
            status, _ = self.sm.composite.place_on_shelf_fallback(
                self._shelf_heights_for(obj.name),
                self.cfg.attempts,
                scan=not self.state.cabinet_scan_fresh,
            )
            self.state.cabinet_scan_fresh = True
        else:
            status = self.sm.manipulation.place()
        if status == Status.EXECUTION_SUCCESS:
            obj.status = "placed"
            self.state.holding = None
            self.sm.hri.publish_display_capture(
                f"Placed {obj.name} at {dest}",
                data={"object": obj.name, "location": dest},
            )
        return status

    def pick_breakfast_item(self, item: BreakfastItem) -> Status:
        name = item.name
        if self.spec(item.name).source == "cabinet":
            status = self.sm.composite.pick_from_shelf(
                name,
                self.shelf_heights,
                self.cfg.attempts,
                self.cfg.use_vision_confirmation,
                self.cfg.use_grasp_detector,
                self.cfg.use_octomap_clearing,
                say_name=name,
            )
            self.state.cabinet_scan_fresh = True
        else:
            self.sm.manipulation.move_to_position("table_stare")
            _, dets = self.sm.vision.detect_objects()
            self.sm.composite.announce_objects([d.classname for d in (dets or [])])
            self.say(f"I will pick the {item.name}.")
            status = self.sm.composite.pick_confirmed(
                name, self.cfg.attempts, False, self.cfg.use_grasp_detector
            )
        if status == Status.EXECUTION_SUCCESS:
            item.status = "picked"
            self._picked(item.name)
        return status

    def pour_into_bowl(self, item: BreakfastItem) -> Status:
        self.say(f"Pouring {item.name} into the bowl.")
        for _ in range(self.cfg.pour_attempts):
            status = self.sm.manipulation.pour(
                pour_object_name=item.name,
                pour_container_name="bowl",
                object_already_grasped=True,
            )
            if status == Status.EXECUTION_SUCCESS:
                self.sm.hri.publish_display_capture(f"Poured {item.name} into bowl")
                return status
        return Status.EXECUTION_ERROR

    def place_breakfast_item(self, item: BreakfastItem) -> Status:
        self.say(f"Placing the {item.name}.")
        ref = self.spec(item.name).close_to
        close_to = ref if ref and self._breakfast(ref).status == "placed" else ""
        status = self.sm.manipulation.place(close_to=close_to)
        if status == Status.EXECUTION_SUCCESS:
            item.status = "placed"
            self.state.holding = None
            self.sm.hri.publish_display_capture(f"Placed breakfast item: {item.name}")
        return status

    def end(self):
        self.say("I have completed the pick and place task.", wait=True)
        self.sm.manipulation.move_to_position("nav_pose")

    # ── Loop control ──

    def _breakfast(self, name: str) -> BreakfastItem:
        return next(b for b in self.state.breakfast if b.name == name)

    def next_object(self):
        placed = sum(o.status == "placed" for o in self.state.objects)
        if placed >= self.cfg.max_cleanup_objects or self.elapsed() > self.cfg.cleanup_deadline_s:
            return None
        return next((o for o in self.state.objects if o.status == "pending"), None)

    def object_done(self, obj: TableObject, ok: bool):
        if obj.status == "pending":
            obj.status = "skipped"

    def next_breakfast(self):
        late = self.elapsed() > self.cfg.breakfast_tail_s
        for item in self.state.breakfast:
            if item.status == "pending" and not (late and not self.spec(item.name).essential):
                return item
        return None

    def breakfast_done(self, item: BreakfastItem, ok: bool):
        if item.status == "pending":
            item.status = "skipped"

    # ── Tree ──

    def clean_object(self, obj: TableObject):
        dest = destination(obj.category, self.cfg)
        steps = [
            Action("navigate_to_table", lambda: self.go(self.table, say=False)),
            Action("ensure_gripper_empty", self.ensure_gripper_empty),
            Action("pick_object", lambda: self.pick_object(obj)),
            Action("navigate_to_placement", lambda: self.go(dest, dock=dest != "trashbin")),
        ]
        if dest == "dishwasher" and self.cfg.use_dishwasher and not self.state.dishwasher_open:
            steps.append(Action("check_dishwasher", self.check_dishwasher))
        if dest == "cabinet":
            steps.append(Action("scan_cabinet_shelves", self.scan_cabinet_shelves))
        steps.append(Action("place_object", lambda: self.place_object(obj, dest)))
        return sequence(f"clean_{obj.name}", *steps)

    def serve_item(self, item: BreakfastItem):
        steps = [
            Action("navigate_to_item_source", lambda: self.go(self.spec(item.name).source)),
            Action("ensure_gripper_empty", self.ensure_gripper_empty),
            Action("pick_breakfast_item", lambda: self.pick_breakfast_item(item)),
            Action("navigate_to_dining", lambda: self.go("dinner_table")),
        ]
        if self.spec(item.name).pour and self._breakfast("bowl").status == "placed":
            steps.append(optional(Action("pour_into_bowl", lambda: self.pour_into_bowl(item))))
        steps.append(Action("place_breakfast_item", lambda: self.place_breakfast_item(item)))
        return sequence(f"breakfast_{item.name}", *steps)

    PHASES = ("start", "perceive", "cleanup", "breakfast")

    def build_tree(self, start_at: str = "start"):
        phases = {
            "start": [
                Action("wait_start", self.wait_start),
                Action("start", self.request_chair_removal),
            ],
            "perceive": [
                optional(Action("perceive_table", self.perceive_table)),
                Action("sort_objects", self.sort_objects),
            ],
            "cleanup": [
                ForEach("cleanup_loop", self.next_object, self.clean_object, self.object_done)
            ],
            "breakfast": [
                Action("start_breakfast_prep", lambda: self.say("Now I will prepare breakfast.")),
                ForEach("breakfast", self.next_breakfast, self.serve_item, self.breakfast_done),
            ],
        }
        if start_at not in phases:
            raise ValueError(f"start_at must be one of {self.PHASES}")
        children = []
        for phase in self.PHASES[self.PHASES.index(start_at) :]:
            children += phases[phase]
        return sequence("pick_and_place", *children)

    def past_hard_deadline(self) -> bool:
        return self.elapsed() > self.cfg.hard_deadline_s


class PickAndPlaceTM(Node):
    """ROS entry: params, subtask manager and the tree runner."""

    def __init__(self):
        super().__init__("pickandplace_task_manager")
        mock = self.declare_parameter("mock_areas", "").value
        self.start_at = self.declare_parameter("start_at", "start").value
        state_file = self.declare_parameter("state_file", "").value
        self.step = self.declare_parameter("step", False).value
        self.log_dir = self.declare_parameter("log_dir", "~/.ros/ppc_runs").value

        from task_manager.utils.subtask_manager import (
            SubtaskManager,
        )  # heavy (BAML); kept out of tests

        mock_areas = [a.strip() for a in mock.split(",") if a.strip()]
        self.sm = SubtaskManager(self, task=Task.PICK_AND_PLACE, mock_areas=mock_areas)
        self.sm.composite.enable_tf()
        state = (
            PPCState.from_dict(json.loads(Path(state_file).read_text()))
            if state_file
            else PPCState()
        )
        self.ppc = PickAndPlace(self.sm, PPCConfig(), state, self)
        self.sm.manipulation.move_to_position("nav_pose")
        CLog.fsm(self, "STATE", f"PPC ready (start_at={self.start_at}, mock={mock_areas}).")

    def run(self):
        ppc = self.ppc
        if self.start_at != "start" and ppc.state.started_at is None:
            ppc.state.started_at = ppc.clock()
        log = RunLog(
            self.log_dir,
            snapshot=lambda: asdict(ppc.state),
            # TODO(hri): the PPC display page listens on its own topic, not TASK_STEP_TOPIC
            on_leaf=self.sm.hri.publish_display_step,
            step=self.step,
        )
        runner = TreeRunner(ppc.build_tree(self.start_at), log, ppc.past_hard_deadline)
        result = runner.run(spin=lambda: rclpy.spin_once(self, timeout_sec=0.0))
        if result.name == "INVALID":
            CLog.fsm(self, "TIMER", "Hard deadline reached, stopping.", level="warn")
        self.sm.hri.publish_display_step("end")
        ppc.end()
        CLog.fsm(self, "TIMER", f"Total {ppc.elapsed():.1f}s — log in {log.dir}")
        for name, seconds in log.report():
            CLog.fsm(self, "TIMER", f"{name}: {seconds:.1f}s")


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceTM()
    try:
        node.run()
    except KeyboardInterrupt:
        CLog.fsm(node, "STATE", "Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
