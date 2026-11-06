#!/usr/bin/env python3
"""Finals routine: patrol the house, report what is out of place, guide people to objects.

The robot drives past every tagged piece of furniture while the semantic map
(vision_general/semantic_map_node.py) fills up on its own from the detector, then
says what is misplaced and answers spoken commands about what it has seen. It never
grasps anything.

A failure anywhere degrades: a viewpoint it cannot reach is skipped, and running out
of time goes straight to the report with whatever it has seen.
"""

import time
from enum import Enum

import rclpy
from rclpy.node import Node
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

ARM_TRAVEL_POSE = "nav_pose"
# Words that end the command phase before the time limit
STOP_WORDS = ("stop", "finish", "that's all", "that is all", "terminar")
HOTWORDS = "where is bring find look stop"


class ExecutionStates(Enum):
    WAIT_START = 0
    PATROL = 10
    REPORT = 20
    COMMANDS = 30
    END = 40


class FinalsTaskManager(Node):
    def __init__(self):
        super().__init__("finals_task_manager")
        self.patrol_time_limit_s = self._param("patrol_time_limit_s", 300.0)
        self.total_time_limit_s = self._param("total_time_limit_s", 600.0)
        self.patrol_mode = self._param("patrol_mode", "full")
        self.nav_retries = self._param("nav_retries", 2)
        self.approach_standoff_m = self._param("approach_standoff_m", 0.65)
        self.max_silent_rounds = self._param("max_silent_rounds", 5)
        mock_areas = self._param("mock_areas", [""])

        self.subtask_manager = SubtaskManager(
            self, task=Task.FINALS, mock_areas=[area for area in mock_areas if area]
        )
        self.state = ExecutionStates.WAIT_START
        self.started_at = 0.0
        Logger.info(self, f"Finals task manager ready, mocking: {mock_areas}")

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    # ------------------------------------------------------------------ helpers

    def _elapsed(self) -> float:
        return time.monotonic() - self.started_at

    def _wait(self, seconds: float) -> None:
        """Wait without blocking the node's callbacks."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    @staticmethod
    def _speakable(name: str) -> str:
        return name.replace("_", " ")

    def _place(self, area: str, sublocation: str) -> str:
        """How to say where something is. Empty when the map does not know."""
        if area and sublocation:
            return f"the {self._speakable(sublocation)} in the {self._speakable(area)}"
        return f"the {self._speakable(area)}" if area else ""

    # ------------------------------------------------------------------- patrol

    def patrol(self, mode: str, time_limit_s: float, max_viewpoints: int = 0) -> int:
        """Stand at each viewpoint long enough for vision to see it.

        Nav only plans the route; this drives it. Returns how many it reached.
        """
        status, route = self.subtask_manager.nav.plan_patrol(
            mode=mode, max_viewpoints=max_viewpoints
        )
        if status != Status.EXECUTION_SUCCESS or not route:
            Logger.error(self, f"No patrol route (mode '{mode}')")
            return 0

        started = time.monotonic()
        reached = 0
        arm_pose = ""
        for step in route:
            if time.monotonic() - started > time_limit_s:
                Logger.warn(self, f"Patrol out of time after {reached}/{len(route)} viewpoints")
                break
            # Never drive with the arm out in a looking pose.
            if arm_pose not in ("", ARM_TRAVEL_POSE):
                self.subtask_manager.manipulation.move_to_position(ARM_TRAVEL_POSE)
            if not self._drive_to(step):
                arm_pose = ARM_TRAVEL_POSE
                continue
            self.subtask_manager.manipulation.move_to_position(step["arm_pose"])
            arm_pose = step["arm_pose"]
            self._wait(step["dwell_s"])
            reached += 1
        self.subtask_manager.manipulation.move_to_position(ARM_TRAVEL_POSE)
        Logger.info(self, f"Patrol done: {reached}/{len(route)} viewpoints")
        return reached

    def _drive_to(self, step: dict) -> bool:
        name = f"{step['area']}/{step['sublocation']}"
        for attempt in range(1 + self.nav_retries):
            status, error = self.subtask_manager.nav.move_to_pose(step["pose"])
            if status == Status.EXECUTION_SUCCESS:
                return True
            Logger.warn(self, f"Could not reach {name} ({error}), attempt {attempt + 1}")
        Logger.error(self, f"Skipping {name}")
        return False

    # ------------------------------------------------------------------- report

    def report_misplaced(self) -> int:
        """Say every object that is not where it belongs. Returns how many."""
        status, misplaced = self.subtask_manager.vision.get_misplaced()
        if status != Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say("I could not check my map of the house.")
            return 0
        if not misplaced:
            self.subtask_manager.hri.say("Everything I saw is in its place.")
            return 0
        self.subtask_manager.hri.say(f"I found {len(misplaced)} things out of place.")
        for obj, expected_area, expected_sublocation in misplaced:
            label = self._speakable(obj.label)
            found = self._place(obj.area, obj.sublocation)
            expected = self._place(expected_area, expected_sublocation)
            self.subtask_manager.hri.say(f"The {label} is on {found}. It belongs on {expected}.")
        return len(misplaced)

    # ----------------------------------------------------------------- commands

    def handle_command(self, text: str) -> None:
        """Answer one spoken command, and guide to the object when there is one."""
        status, answer, objects = self.subtask_manager.vision.resolve_command(text)
        if status != Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say("Sorry, I could not check my map of the house.")
            return
        if answer == "not_found":
            self.subtask_manager.hri.say("I have not seen that. I will take another look.")
            self.patrol("revisit", self.patrol_time_limit_s, max_viewpoints=3)
            status, answer, objects = self.subtask_manager.vision.resolve_command(text)
            if status != Status.EXECUTION_SUCCESS or answer == "not_found":
                self.subtask_manager.hri.say("Sorry, I could not find it anywhere.")
                return
        if answer == "list":
            self._say_list(objects)
            return
        self._guide_to(objects[0])

    def _say_list(self, objects: list) -> None:
        if not objects:
            self.subtask_manager.hri.say("I did not see anything there.")
            return
        names = ", ".join(self._speakable(obj.label) for obj in objects)
        self.subtask_manager.hri.say(f"I saw: {names}.")

    def _guide_to(self, obj) -> None:
        label = self._speakable(obj.label)
        place = self._place(obj.area, obj.sublocation)
        where = f"on {place}" if place else "nearby"
        self.subtask_manager.hri.say(f"The {label} is {where}. Please follow me.")
        status, error = self.subtask_manager.nav.approach_point(
            obj.position, standoff=self.approach_standoff_m
        )
        if status == Status.EXECUTION_SUCCESS:
            self.subtask_manager.hri.say(f"Here is the {label}.")
        else:
            Logger.error(self, f"Could not reach the {label}: {error}")
            self.subtask_manager.hri.say(f"Sorry, I cannot get to the {label} from here.")

    def command_loop(self) -> None:
        """Listen until told to stop, the time runs out, or nobody speaks for a while."""
        self.subtask_manager.hri.say("What are you looking for?")
        silent_rounds = 0
        while self._elapsed() < self.total_time_limit_s and silent_rounds < self.max_silent_rounds:
            status, text, _ = self.subtask_manager.hri.hear(hotwords=HOTWORDS)
            if status != Status.EXECUTION_SUCCESS or not text:
                silent_rounds += 1
                continue
            silent_rounds = 0
            if any(word in text.lower() for word in STOP_WORDS):
                return
            self.handle_command(text)
            self.subtask_manager.hri.say("Anything else?")

    # ---------------------------------------------------------------------- run

    def run(self) -> None:
        hri, vision = self.subtask_manager.hri, self.subtask_manager.vision

        self.state = ExecutionStates.WAIT_START
        hri.reset_task_status()
        hri.say("Press the start button when you are ready.", wait=False)
        while not hri.start_button_clicked and not self.subtask_manager.get_mocked_areas():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.started_at = time.monotonic()
        if vision.reset_semantic_map() != Status.EXECUTION_SUCCESS:
            Logger.warn(self, "Could not reset the semantic map, it may hold a previous run")

        self.state = ExecutionStates.PATROL
        hri.say("I will look around the house.", wait=False)
        self.patrol(self.patrol_mode, self.patrol_time_limit_s)

        self.state = ExecutionStates.REPORT
        self.report_misplaced()

        self.state = ExecutionStates.COMMANDS
        self.command_loop()

        self.state = ExecutionStates.END
        hri.say("I am done. Thank you.")


def main(args=None):
    rclpy.init(args=args)
    node = FinalsTaskManager()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
