"""Dispatch and the pick -> place handoff.

ManipulationCore is a ROS node, so these drive a bare instance with only the
attributes the methods under test touch.
"""

import pytest

from frida_interfaces.msg import ManipulationTask

from pick_and_place.manipulation_core import ManipulationCore
from pick_and_place.pipelines.errors import PickHardwareError
from pick_and_place.pipelines.strategies import PickOutcome

from fakes import FakeArm, FakeLogger, FakePerception, make_pose


@pytest.fixture
def core():
    node = ManipulationCore.__new__(ManipulationCore)
    node.arm = FakeArm()
    node.perception = FakePerception()
    node.strategies = {}
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node._last_pick = PickOutcome()
    node._pipelines = {
        ManipulationTask.PICK: node._run_pick,
        ManipulationTask.PICK_CLOSEST: node._run_pick_closest,
        ManipulationTask.PLACE: node._run_place,
        ManipulationTask.POUR: node._run_pour,
    }
    return node


class _Request:
    """A ManipulationAction goal request, only the fields the node reads."""

    def __init__(self, task_type=ManipulationTask.PICK, **params):
        self.task_type = task_type
        self.scan_environment = params.pop("scan_environment", False)
        self.pick_params = type(
            "P",
            (),
            {
                "object_name": "cup",
                "object_point": None,
                "min_distance": 0.0,
                "max_distance": 1.0,
                "in_configuration": False,
            },
        )()
        self.place_params = type("P", (), {"is_shelf": False})()
        self.pour_params = type(
            "P",
            (),
            {
                "object_name": "cereal",
                "bowl_name": "bowl",
                "object_already_grasped": False,
            },
        )()


# --- task dispatch -----------------------------------------------------------


def test_every_task_type_has_a_pipeline(core):
    for task in (
        ManipulationTask.PICK,
        ManipulationTask.PICK_CLOSEST,
        ManipulationTask.PLACE,
        ManipulationTask.POUR,
    ):
        assert task in core._pipelines


def test_pick_closest_fails_loudly_instead_of_raising(core):
    """It used to reference an unbound local and raise NameError."""
    assert core._run_pick_closest(_Request()) is False
    assert any("not implemented" in m for m in core._logger.error_messages)


# --- the pick -> place handoff ------------------------------------------------


def test_a_successful_pick_is_remembered_for_the_place(core, monkeypatch):
    outcome = PickOutcome(pick_pose=make_pose(), object_pick_height=0.12)
    monkeypatch.setattr(
        "pick_and_place.pipelines.pick.execute", lambda *a, **k: (True, outcome)
    )
    assert core._run_pick(_Request()) is True
    assert core._last_pick is outcome


def test_a_failed_pick_clears_the_remembered_outcome(core, monkeypatch):
    core._last_pick = PickOutcome(pick_pose=make_pose(), object_pick_height=0.12)
    monkeypatch.setattr(
        "pick_and_place.pipelines.pick.execute",
        lambda *a, **k: (False, PickOutcome()),
    )
    assert core._run_pick(_Request()) is False
    assert core._last_pick.pick_pose is None


def test_a_raising_pour_does_not_leave_a_stale_outcome_behind(core, monkeypatch):
    """A later place would otherwise use a previous task's drop height."""
    core._last_pick = PickOutcome(pick_pose=make_pose(), object_pick_height=0.30)

    def boom(*args, **kwargs):
        raise PickHardwareError("planner died")

    monkeypatch.setattr("pick_and_place.pipelines.pour.execute", boom)
    with pytest.raises(PickHardwareError):
        core._run_pour(_Request(ManipulationTask.POUR))
    assert core._last_pick.pick_pose is None


def test_the_place_consumes_the_remembered_pick(core, monkeypatch):
    captured = {}
    remembered = PickOutcome(pick_pose=make_pose(), object_pick_height=0.2)
    core._last_pick = remembered
    monkeypatch.setattr(
        "pick_and_place.pipelines.place.execute",
        lambda arm, perc, params, outcome: captured.setdefault("outcome", outcome)
        is None,
    )
    core._run_place(_Request(ManipulationTask.PLACE))
    assert captured["outcome"] is remembered
