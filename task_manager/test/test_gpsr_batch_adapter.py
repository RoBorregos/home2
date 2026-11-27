"""Parked checks for the ROS-facing batch adapter. No node or hardware is started."""

import sys
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import gpsr_task_manager
from gpsr_task_manager import GPSRTM
from task_manager.gpsr.merger import PlanAction, issued_order_plan, merge
from task_manager.gpsr.recovery import GripperState
from task_manager.gpsr.status import Status
from task_manager.gpsr.timeouts import timeout_for
from task_manager.subtask_managers.gpsr_single_tasks import GPSRSingleTask


def action(kind, cmd, idx, **fields):
    raw = SimpleNamespace(action=kind, **fields)
    return PlanAction(raw, cmd, idx, fields.get("location_to_go"), False, False)


def test_report_uses_only_verified_earlier_facts_from_its_own_command():
    hri = Mock()
    hri.answer_with_context.return_value = Status.EXECUTION_SUCCESS, "Two cups."
    hri.say.return_value = Status.EXECUTION_SUCCESS
    single = GPSRSingleTask.__new__(GPSRSingleTask)
    single.subtask_manager = SimpleNamespace(hri=hri)
    node = SimpleNamespace(
        _facts={},
        _completed=set(),
        gpsr_individual_tasks=single,
        gpsr_tasks=SimpleNamespace(),
        subtask_manager=single.subtask_manager,
        _capture_command=Mock(),
        _trace=Mock(),
        trial_log=None,
        _describe_action=GPSRTM._describe_action,
        _gripper_state=Mock(return_value=GripperState.EMPTY),
        get_logger=Mock(),
    )
    for cmd, idx, status, value in [
        (0, 1, Status.EXECUTION_SUCCESS, "two cups"),
        (1, 1, Status.EXECUTION_SUCCESS, "five cups"),
        (0, 2, Status.EXECUTION_ERROR, "bad detection"),
        (0, 8, Status.EXECUTION_SUCCESS, "later observation"),
    ]:
        GPSRTM._on_action_complete(node, action("count", cmd, idx), status, value)
    report = action(
        "say_with_context", 0, 4, previous_command_info=["count"], user_instruction="How many?"
    )
    GPSRTM._on_action_start(node, report)
    assert single.say_with_context(report.action) == (Status.EXECUTION_SUCCESS, "Two cups.")
    hri.answer_with_context.assert_called_once_with("How many?", "count: two cups")
    hri.query_command_history.assert_not_called()
    report.source_cmd = 2
    GPSRTM._on_action_start(node, report)
    assert single.say_with_context(report.action)[0] == Status.TARGET_NOT_FOUND
    assert hri.say.call_count == 1


def test_revisit_reserves_route_home_after_a_failed_observation():
    node = SimpleNamespace(_at_instruction_point=False, _path_distance=Mock(return_value=2))
    steps = [action("go_to", 0, -1, location_to_go="couch"), action("count", 0, 1)]
    assert GPSRTM._estimate_revisit(node, steps) == 2 * timeout_for("go_to") + timeout_for("count")
    node._path_distance.side_effect = lambda src, dst: None if src == "couch" else 1
    assert GPSRTM._estimate_revisit(node, steps) is None


@pytest.mark.parametrize(
    "goal_status, at_start, returned",
    [
        (Status.EXECUTION_SUCCESS, True, True),
        (Status.EXECUTION_SUCCESS, False, False),
        (Status.EXECUTION_ERROR, True, False),
    ],
)
def test_return_requires_navigation_success_and_verified_pose(goal_status, at_start, returned):
    node = SimpleNamespace(
        navigate_to=Mock(return_value=goal_status),
        subtask_manager=SimpleNamespace(hri=Mock()),
        _at_start_pose=Mock(return_value=at_start),
        _trace=Mock(),
        trial_log=None,
    )
    assert GPSRTM._return_to_start(node, time.monotonic() + 10) is returned
    node.navigate_to.assert_called_once()
    assert GPSRTM._return_to_start(node, time.monotonic() - 1) is False
    node.navigate_to.assert_called_once()


def test_return_that_finishes_late_is_logged_as_late(monkeypatch):
    monkeypatch.setattr(gpsr_task_manager.time, "monotonic", Mock(side_effect=[0.0, 11.0]))
    node = SimpleNamespace(
        navigate_to=Mock(return_value=Status.EXECUTION_SUCCESS),
        subtask_manager=SimpleNamespace(hri=Mock()),
        _at_start_pose=Mock(return_value=True),
        _trace=Mock(),
        trial_log=None,
    )
    assert GPSRTM._return_to_start(node, 10.0) is False
    node._trace.assert_called_with(
        "return_end", returned=False, arrived=True, status="EXECUTION_SUCCESS", pose=None
    )


def test_grasp_feedback_has_a_freshness_limit(monkeypatch):
    monkeypatch.setattr(gpsr_task_manager.time, "monotonic", lambda: 10.0)
    node = SimpleNamespace(_grasp_reading=None)
    assert GPSRTM._gripper_state(node) is GripperState.UNCERTAIN
    node._grasp_reading = (7.0, True)
    assert GPSRTM._gripper_state(node) is GripperState.UNCERTAIN
    GPSRTM._on_grasp_reading(node, SimpleNamespace(object_detected=True))
    assert GPSRTM._gripper_state(node) is GripperState.OCCUPIED
    GPSRTM._on_grasp_reading(node, SimpleNamespace(object_detected=False))
    assert GPSRTM._gripper_state(node) is GripperState.EMPTY


def test_issued_order_reuses_shared_drive_and_preserves_source_indices():
    drive = SimpleNamespace(action="go_to", location_to_go="table")
    commands = [
        SimpleNamespace(commands=actions)
        for actions in (
            [drive, SimpleNamespace(action="count")],
            [drive],
            [drive],
        )
    ]
    plan = issued_order_plan(merge(commands))
    assert [(a.source_cmd, a.source_idx) for a in plan.actions] == [(0, 0), (0, 1)]
    assert plan.actions[0].navigation_goals == ((1, 0), (2, 0))
    assert len(plan.fallback) == 3


def test_plan_only_does_not_build_or_dispatch_the_tree(monkeypatch):
    build_tree = Mock(side_effect=AssertionError("plan-only must not build the execution tree"))
    monkeypatch.setattr(gpsr_task_manager, "build_tree", build_tree)
    node = SimpleNamespace(
        trial_deadline=None,
        elapsed_before_batch_s=0.0,
        batch_order="merged",
        plan_only=True,
        recovery_policy=SimpleNamespace(value="skip-only"),
        replan_on_failure=False,
        use_nav_paths=False,
        nominal_speed=1.0,
        holding_penalty=0.0,
        navigation_overhead=0.0,
        skill_durations={},
        _trace=Mock(),
        _completed=set(),
        _facts={},
        _planner_options=dict,
        batched_commands=[],
        _at_instruction_point=False,
        get_logger=Mock(),
        _spoken_merged_plan=Mock(return_value="the plan"),
        subtask_manager=SimpleNamespace(hri=Mock()),
    )
    assert GPSRTM._execute_interleaved_batch(node) is False
    build_tree.assert_not_called()
