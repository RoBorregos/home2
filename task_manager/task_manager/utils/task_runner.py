"""
Shared base for task managers.

Every task manager had grown its own copy of the same five things: an identical
``main()``, a ``navigate_to`` wrapper, a ``timeout()`` spin-sleep, ``_track_state_change``,
and a start-button busy-wait. They had drifted — different return types, one variant
using ``time.sleep`` (which blocks callbacks), step reporting in three incompatible
styles, and only three of eight managers publishing steps at all.

This collects them once so a task manager holds task logic and nothing else.
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node

from task_manager.gpsr.bt_decorators import Budget
from task_manager.gpsr.timeouts import TASK_BUDGET_S
from task_manager.utils.colored_logger import CLog
from task_manager.utils.run_log import RunLog
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task


class TaskRunner(Node):
    """Base node wiring a task's subtask managers, time budget and run log."""

    def __init__(
        self,
        node_name: str,
        task: Task,
        run_name: str,
        budget_s: float = TASK_BUDGET_S,
        mock_areas: Optional[list] = None,
    ):
        super().__init__(node_name)
        self.task = task
        self.run_name = run_name
        self.subtask_manager = SubtaskManager(self, task=task, mock_areas=mock_areas or [])
        self.budget = Budget(total_s=budget_s)
        self.running_task = True
        self.current_state = ""
        self.previous_state = ""
        self.state_times: dict[str, float] = {}
        self.state_start_time = time.time()
        self.current_attempts = 0

    # ---------------- lifecycle ----------------

    def wait_for_start(self, wait_for_door: bool = True) -> None:
        """Block until the start button and (optionally) the open door, then anchor the clock."""
        self.say("Waiting for the start signal.", wait=False)
        while rclpy.ok() and not self.subtask_manager.hri.start_button_clicked:
            rclpy.spin_once(self, timeout_sec=0.1)

        if wait_for_door:
            self.say("Waiting for the door to open.", wait=False)
            while rclpy.ok():
                status, _ = self.subtask_manager.nav.check_door()
                if status == Status.EXECUTION_SUCCESS:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)

        # the scored window starts here, not at node construction
        self.budget.start()
        run_id = RunLog.start(self.run_name)
        CLog.fsm(self, "STATE", f"Start signal. Run log: {run_id}", level="success")

    def finish(self, score: Optional[int] = None) -> None:
        """Report timings, close the run log and leave the arm safe."""
        total = self.budget.elapsed()
        CLog.fsm(self, "TIMER", "=== FINAL TIMING REPORT ===")
        CLog.fsm(self, "TIMER", f"Total task time: {total:.2f}s")
        for state, spent in sorted(self.state_times.items(), key=lambda kv: kv[1], reverse=True):
            share = (spent / total * 100) if total > 0 else 0.0
            CLog.fsm(self, "TIMER", f"{state}: {spent:.2f}s ({share:.1f}%)")
        RunLog.note("state_times", total_s=round(total, 2), states=dict(self.state_times))
        RunLog.finish(score=score)
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        self.running_task = False

    # ---------------- helpers every manager had its own copy of ----------------

    def set_state(self, new_state: str) -> None:
        """Record time in the previous state and mirror the new one to the display."""
        now = time.time()
        if self.previous_state:
            spent = now - self.state_start_time
            self.state_times[self.previous_state] = (
                self.state_times.get(self.previous_state, 0.0) + spent
            )
            CLog.fsm(self, "TIMER", f"State '{self.previous_state}' took {spent:.2f}s")
        self.previous_state = new_state
        self.state_start_time = now
        self.current_state = new_state
        self.current_attempts = 0
        RunLog.note("state", name=new_state)
        try:
            self.subtask_manager.hri.publish_display_step(new_state.lower())
        except Exception as error:  # noqa: BLE001 — the display must never stop a run
            self.get_logger().warning(f"publish_display_step failed: {error}")

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True) -> Status:
        """Stow the arm, announce the move, then drive. Returns the nav Status."""
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        if say:
            target = (sublocation or location).replace("_", " ")
            self.say(f"I am going to the {target}.", wait=False)
        status, _ = self.subtask_manager.nav.move_to_location(location, sublocation)
        return status

    def say(self, text: str, wait: bool = True) -> None:
        try:
            self.subtask_manager.hri.say(text, wait=wait)
        except Exception as error:  # noqa: BLE001 — speech failure must not stop a run
            self.get_logger().warning(f"say failed: {error}")

    def timeout(self, duration: float = 2.0) -> None:
        """Spin ROS for a while. Never time.sleep here — it starves callbacks."""
        start = time.time()
        while rclpy.ok() and (time.time() - start) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

    def out_of_time(self, reserve_s: float = 0.0) -> bool:
        """True when the run budget is spent, keeping `reserve_s` for shutdown."""
        return self.budget.remaining() <= reserve_s

    # ---------------- entry point ----------------

    def run(self) -> None:
        """Override with the task's step. Called repeatedly by spin_task."""
        raise NotImplementedError


def spin_task(node_factory, args=None) -> None:
    """The identical main() every task manager used to carry its own copy of."""
    rclpy.init(args=args)
    node = node_factory()
    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
