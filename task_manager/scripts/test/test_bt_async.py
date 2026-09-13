#!/usr/bin/env python3
"""
Offline tests for the asynchronous behaviour-tree executor.

Proves the thing that was broken before: a leaf now returns RUNNING, so the Timeout
and Deadline decorators actually preempt work instead of being inert decoration.

ROS-free — needs only py_trees.
"""

import os
import sys
import time
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))


def _stub_rclpy() -> None:
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

import py_trees  # noqa: E402

from task_manager.gpsr.bt_builder import build_tree  # noqa: E402
from task_manager.gpsr.bt_decorators import Budget, Deadline  # noqa: E402
from task_manager.gpsr.leaf_behaviours import ActionLeaf  # noqa: E402
from task_manager.gpsr.merger import InterleavedPlan, PlanAction  # noqa: E402
from task_manager.gpsr.skill_runner import SkillRunner  # noqa: E402
from task_manager.utils.status import Status  # noqa: E402


class FakeAction:
    """Stands in for a BAML command object: dispatch only needs `.action`."""

    def __init__(self, kind: str, **fields):
        self.action = kind
        for key, value in fields.items():
            setattr(self, key, value)


def plan_action(kind: str, cmd: int = 0, idx: int = 0) -> PlanAction:
    return PlanAction(
        action=FakeAction(kind),
        source_cmd=cmd,
        source_idx=idx,
        location=None,
        requires_gripper=False,
        releases_gripper=False,
    )


class Handlers:
    """Subtask-manager stand-in whose method names match action kinds."""

    def __init__(self):
        self.calls = []

    def go_to(self, command):
        self.calls.append("go_to")
        return Status.EXECUTION_SUCCESS, "arrived"

    def pick_object(self, command):
        self.calls.append("pick_object")
        return Status.EXECUTION_SUCCESS, "picked"

    def slow_action(self, command):
        self.calls.append("slow_action")
        time.sleep(1.5)
        return Status.EXECUTION_SUCCESS, "eventually"

    def failing(self, command):
        self.calls.append("failing")
        return Status.TARGET_NOT_FOUND, None


def tick_until_done(root, limit_s: float = 10.0):
    """Drive the tree the way the task manager's loop does."""
    terminal = (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE)
    start = time.time()
    ticks = 0
    while time.time() - start < limit_s:
        root.tick_once()
        ticks += 1
        if root.status in terminal:
            return root.status, ticks
        time.sleep(0.01)
    raise AssertionError("tree never reached a terminal status")


def test_leaf_returns_running() -> None:
    """The core fix: update() must not block the tick."""
    handlers = Handlers()
    runner = SkillRunner()
    leaf = ActionLeaf(plan_action("slow_action"), [handlers], runner=runner)
    leaf.setup()

    leaf.tick_once()
    assert leaf.status == py_trees.common.Status.RUNNING, leaf.status

    # the tick returned immediately even though the skill takes 1.5 s
    status, ticks = tick_until_done(leaf)
    assert status == py_trees.common.Status.SUCCESS, status
    assert ticks > 5, f"expected many ticks while running, got {ticks}"
    runner.shutdown()
    print("✓ leaf returns RUNNING while the skill works")


def test_timeout_now_fires() -> None:
    """Before the async change this decorator could never trip."""
    handlers = Handlers()
    runner = SkillRunner()
    leaf = ActionLeaf(plan_action("slow_action"), [handlers], runner=runner)
    guarded = py_trees.decorators.Timeout(name="to", child=leaf, duration=0.3)
    guarded.setup()

    status, _ = tick_until_done(guarded)
    assert status == py_trees.common.Status.FAILURE, status
    assert runner.abandoned_count == 1, runner.abandoned_count
    runner.shutdown()
    print("✓ per-action Timeout preempts a long skill")


def test_deadline_fires_and_reports() -> None:
    budget = Budget(total_s=0.4)
    budget.start()
    fired = []

    handlers = Handlers()
    runner = SkillRunner()
    leaf = ActionLeaf(plan_action("slow_action"), [handlers], runner=runner)
    guarded = Deadline(
        child=leaf, remaining_s=budget.remaining, on_expire=lambda: fired.append(True)
    )
    guarded.setup()

    status, _ = tick_until_done(guarded)
    assert status == py_trees.common.Status.FAILURE, status
    assert fired == [True], fired
    runner.shutdown()
    print("✓ Deadline trips on the run budget and fires its hook once")


def test_budget_unanchored_never_trips() -> None:
    budget = Budget(total_s=420.0)
    assert budget.remaining() == 420.0
    assert not budget.expired()
    budget.start()
    assert budget.remaining() < 420.0
    print("✓ budget is inert until anchored at door-open")


def test_build_tree_happy_path() -> None:
    handlers = Handlers()
    plan = InterleavedPlan(
        actions=[plan_action("go_to", 0, 0), plan_action("pick_object", 0, 1)],
        fallback=[],
    )
    completed = []
    root = build_tree(
        plan,
        subtask_handlers=[handlers],
        on_action_complete=lambda pa, st, res: completed.append((pa.source_idx, st)),
        remaining_s=Budget(total_s=60.0).remaining,
    )
    root.setup_with_descendants()

    status, _ = tick_until_done(root)
    assert status == py_trees.common.Status.SUCCESS, status
    assert handlers.calls == ["go_to", "pick_object"], handlers.calls
    assert [idx for idx, _ in completed] == [0, 1], completed
    print("✓ build_tree runs a plan end to end under a deadline")


def test_build_tree_falls_back() -> None:
    handlers = Handlers()
    # the fallback keeps its go_to because real work follows it in the same segment
    plan = InterleavedPlan(
        actions=[plan_action("failing", 0, 0)],
        fallback=[[plan_action("go_to", 0, 0), plan_action("pick_object", 0, 1)]],
    )
    root = build_tree(plan, subtask_handlers=[handlers], retry_count=1)
    root.setup_with_descendants()

    status, _ = tick_until_done(root)
    assert status == py_trees.common.Status.SUCCESS, status
    assert handlers.calls == ["failing", "go_to", "pick_object"], handlers.calls
    print("✓ interleaved failure falls through to the sequential branch")


def test_lone_go_to_is_skipped_on_resume() -> None:
    """A fallback go_to with no remaining work in its segment must not drive the robot."""
    handlers = Handlers()
    plan = InterleavedPlan(
        actions=[plan_action("failing", 0, 0)],
        fallback=[[plan_action("go_to", 0, 0)]],
    )
    root = build_tree(plan, subtask_handlers=[handlers], retry_count=1)
    root.setup_with_descendants()

    status, _ = tick_until_done(root)
    assert status == py_trees.common.Status.SUCCESS, status
    assert handlers.calls == ["failing"], handlers.calls
    print("✓ pointless fallback navigation is skipped")


if __name__ == "__main__":
    test_leaf_returns_running()
    test_timeout_now_fires()
    test_deadline_fires_and_reports()
    test_budget_unanchored_never_trips()
    test_build_tree_happy_path()
    test_build_tree_falls_back()
    test_lone_go_to_is_skipped_on_resume()
    print("\nAll async behaviour-tree tests passed.")
