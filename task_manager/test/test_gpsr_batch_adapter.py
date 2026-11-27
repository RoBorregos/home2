"""Parked checks for the ROS-facing batch adapter. No node or hardware is started."""

import sys
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gpsr_task_manager import GPSRTM
from task_manager.gpsr.merger import PlanAction
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
    "goal_status, returned",
    [
        (Status.EXECUTION_SUCCESS, True),
        (Status.EXECUTION_ERROR, False),
    ],
)
def test_return_follows_navigation_result(goal_status, returned):
    node = SimpleNamespace(
        navigate_to=Mock(return_value=goal_status), subtask_manager=SimpleNamespace(hri=Mock())
    )
    assert GPSRTM._return_to_start(node, time.monotonic() + 10) is returned
    node.navigate_to.assert_called_once()
    assert GPSRTM._return_to_start(node, time.monotonic() - 1) is False
    node.navigate_to.assert_called_once()
