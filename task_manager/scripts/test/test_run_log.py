#!/usr/bin/env python3
"""
Offline test for the run log and the @measured decorator.

ROS-free by design: run it with plain python3, no workspace sourcing needed.
"""

import json
import os
import sys
import tempfile
import types

os.environ["FRIDA_RUN_LOG_DIR"] = tempfile.mkdtemp(prefix="frida_runs_test_")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))


def _stub_rclpy() -> None:
    """Let decorators.py import without a ROS install so @measured is testable in CI."""
    if "rclpy" in sys.modules:
        return
    rclpy = types.ModuleType("rclpy")
    action = types.ModuleType("rclpy.action")
    client = types.ModuleType("rclpy.client")
    action.ActionClient = type("ActionClient", (), {})
    client.Client = type("Client", (), {})
    rclpy.action = action
    rclpy.client = client
    sys.modules.update({"rclpy": rclpy, "rclpy.action": action, "rclpy.client": client})


_stub_rclpy()

from task_manager.utils.decorators import measured  # noqa: E402
from task_manager.utils.run_log import RunLog, _status_name  # noqa: E402


class FakeStatus:
    """Stand-in for utils.status.Status so the test needs no rclpy import."""

    def __init__(self, name: str):
        self.name = name


def read_rows(run_id: str) -> list:
    path = os.path.join(os.environ["FRIDA_RUN_LOG_DIR"], f"{run_id}.jsonl")
    with open(path, encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def test_run_lifecycle() -> None:
    run_id = RunLog.start("pick_and_place")
    assert run_id and run_id.startswith("pick_and_place_"), run_id

    RunLog.record("go_to", FakeStatus("EXECUTION_SUCCESS"), 31.2, {"location": "dining_table"})
    RunLog.record("pick", FakeStatus("TARGET_NOT_FOUND"), 42.7, {"object_class": "cutlery"})
    RunLog.note("objective_selected", objective="pick_place", ev=2.0)
    RunLog.finish(score=170)

    rows = read_rows(run_id)
    events = [row["event"] for row in rows]
    assert events == ["run_start", "skill", "skill", "objective_selected", "run_end"], events

    pick = rows[2]
    assert pick["skill"] == "pick"
    assert pick["status"] == "TARGET_NOT_FOUND"
    assert pick["duration_s"] == 42.7
    assert pick["context"] == {"object_class": "cutlery"}
    assert all(row["run_id"] == run_id for row in rows)
    assert rows[-1]["score"] == 170
    print("✓ run lifecycle")


def test_status_normalization() -> None:
    assert _status_name(FakeStatus("TIMEOUT")) == "TIMEOUT"
    assert _status_name(True) == "EXECUTION_SUCCESS"
    assert _status_name(False) == "EXECUTION_ERROR"
    assert _status_name(None) == "None"
    print("✓ status normalization")


def test_record_without_run_is_noop() -> None:
    RunLog.record("pick", FakeStatus("EXECUTION_SUCCESS"), 1.0)
    assert RunLog.run_id() is None
    assert RunLog.elapsed() == 0.0
    print("✓ no-op outside a run")


class FakeSkills:
    """Minimal stand-in for a subtask manager carrying @measured skills."""

    mock_data = False

    @measured("pick", context=lambda self, obj, **kw: {"object_class": obj})
    def pick_object(self, obj: str):
        return FakeStatus("EXECUTION_SUCCESS"), f"picked {obj}"

    @measured()
    def go_to(self, location: str):
        return FakeStatus("TIMEOUT")

    @measured("explode")
    def broken(self):
        raise ValueError("boom")


def test_measured_decorator() -> None:
    run_id = RunLog.start("measured")
    skills = FakeSkills()

    status, payload = skills.pick_object("cutlery")
    assert payload == "picked cutlery"
    assert skills.go_to("kitchen").name == "TIMEOUT"

    try:
        skills.broken()
        raise AssertionError("exception should propagate")
    except ValueError:
        pass

    RunLog.finish()
    rows = [row for row in read_rows(run_id) if row["event"] == "skill"]
    assert [row["skill"] for row in rows] == ["pick", "go_to", "explode"], rows

    # tuple returns unpack to their Status, context callables are applied
    assert rows[0]["status"] == "EXECUTION_SUCCESS"
    assert rows[0]["context"] == {"object_class": "cutlery"}
    # bare Status returns work, and the name defaults to the function's
    assert rows[1]["status"] == "TIMEOUT"
    # a raising skill is still recorded, then re-raised
    assert rows[2]["status"] == "EXCEPTION:ValueError"
    assert all(row["duration_s"] >= 0 for row in rows)

    # functools.wraps keeps introspection working for the skill registry
    assert FakeSkills.go_to.__name__ == "go_to"
    assert list(FakeSkills.pick_object.__annotations__) == ["obj"]
    print("✓ measured decorator")


def test_logging_never_raises() -> None:
    RunLog.start("laundry")
    RunLog._path = "/nonexistent-dir/should-not-explode.jsonl"
    RunLog.record("pick", FakeStatus("EXECUTION_SUCCESS"), 1.0)
    RunLog._enabled = True
    print("✓ write failures are swallowed")


if __name__ == "__main__":
    test_run_lifecycle()
    test_status_normalization()
    test_record_without_run_is_noop()
    test_measured_decorator()
    test_logging_never_raises()
    print("\nAll run-log tests passed.")
