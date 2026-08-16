#!/usr/bin/env python3

"""
Task Manager for GPSR task of Robocup @Home 2026
"""

import time
from datetime import datetime

import rclpy
from frida_constants.hri_constants import (
    GPSR_COMMAND_INDEX_TOPIC,
    GPSR_TASK_STEP_TOPIC,
    ANSWER_PUBLISHER,
)
from frida_constants.vision_constants import IMAGE_ORIENTED_TOPIC
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener
import py_trees

from task_manager.gpsr.bt_builder import build_tree, render_tree_ascii
from task_manager.gpsr.merger import merge
from task_manager.gpsr.timeouts import GLOBAL_BUDGET_S
from task_manager.subtask_managers.gpsr_single_tasks import GPSRSingleTask
from task_manager.subtask_managers.gpsr_tasks import GPSRTask

# from task_manager.subtask_managers.gpsr_test_commands import get_gpsr_comands
from task_manager.utils.baml_client.types import CommandListLLM
from task_manager.utils.colored_logger import CLog
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3
MAX_COMMANDS = 3
BATCH_SIZE = 3


def confirm_command(interpreted_text, target_info):
    return f"Is your command: {target_info}? Yes or no?"


def search_command(command, objects: list[object]):
    for object in objects:
        if hasattr(object, command):
            method = getattr(object, command)
            if callable(method):
                return method
    return None


class GPSRTM(Node):
    """Class to manage the GPSR task"""

    class TaskStates:
        WAITING_FOR_BUTTON = "WAITING_FOR_BUTTON"
        START = "START"
        WAITING_FOR_COMMAND = "WAITING_FOR_COMMAND"
        EXECUTING_COMMAND = "EXECUTING_COMMAND"
        FINISHED_COMMAND = "FINISHED_COMMAND"
        PLAN_AND_EXECUTE_BATCH = "PLAN_AND_EXECUTE_BATCH"
        DONE = "DONE"
        WAIT_BUTTON_COMMAND = "WAIT_BUTTON_COMMAND"

    def __init__(self):
        """Initialize the node"""
        super().__init__("gpsr_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.GPSR, mock_areas=[])
        self.gpsr_tasks = GPSRTask(self.subtask_manager)
        self.gpsr_individual_tasks = GPSRSingleTask(self.subtask_manager)
        self._command_index_pub = self.create_publisher(Int32, GPSR_COMMAND_INDEX_TOPIC, 10)

        self.current_state = (
            # GPSRTM.TaskStates.START
            GPSRTM.TaskStates.WAITING_FOR_BUTTON
            # GPSRTM.TaskStates.WAITING_FOR_COMMAND
            # GPSRTM.TaskStates.EXECUTING_COMMAND
        )
        self.running_task = True
        self.current_hear_attempt = 0
        self.executed_commands = 0
        # self.commands = get_gpsr_comands("custom")
        self.commands = []

        if isinstance(self.commands, dict):
            self.commands = CommandListLLM(**self.commands).commands
        elif isinstance(self.commands, CommandListLLM):
            self.commands = self.commands.commands

        # Batch-mode state for the +200 pt interleaved-execution bonus.
        self.declare_parameter("interleave_enabled", True)
        self.declare_parameter("batch_size", BATCH_SIZE)
        self.declare_parameter("test_mode", False)
        self.declare_parameter(
            "test_commands",
            [
                "tell me how many people in the bathroom are wearing red blouses",
                "tell the pose of the person at the sofa to the person at the lamp",
                "go to the storage rack then find a pringles and grasp it and bring it to the standing person in the living room",
            ],
        )

        self.interleave_enabled = bool(
            self.get_parameter("interleave_enabled").get_parameter_value().bool_value
        )
        self.batch_size = int(
            self.get_parameter("batch_size").get_parameter_value().integer_value or BATCH_SIZE
        )
        self.test_mode = bool(self.get_parameter("test_mode").get_parameter_value().bool_value)
        self.test_nl_commands = list(
            self.get_parameter("test_commands").get_parameter_value().string_array_value
        )

        if self.test_mode:
            self.current_state = GPSRTM.TaskStates.WAITING_FOR_COMMAND

        self.batched_commands: list = []
        self._location_cache: dict[str, tuple[float, float] | None] = {}
        # Live areas table (poses) fetched once per planning session via the
        # retrieve_areas service; falls back to nav.areas_backup when the
        # service is unavailable. Reset alongside _location_cache.
        self._areas: dict | None = None
        # (source_cmd, source_idx) of actions that succeeded in the interleaved
        # branch, so the sequential fallback can resume instead of restarting.
        self._completed: set[tuple[int, int]] = set()

        # TF for resolving robot pose at batch start (used as the planning
        # origin so the merger optimises travel relative to where the robot
        # actually is, not the SLAM map origin (0,0)).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State timing variables
        self.state_start_time = None
        self.state_times = {}
        self.total_start_time = datetime.now()
        self.previous_state = None

        CLog.fsm(self, "STATE", "GPSRTMTaskManager has started.")

    def _track_state_change(self, new_state: str):
        """Track state changes and time spent in each state"""
        current_time = datetime.now()

        if self.previous_state and self.state_start_time:
            time_spent = (current_time - self.state_start_time).total_seconds()
            if self.previous_state in self.state_times:
                self.state_times[self.previous_state] += time_spent
            else:
                self.state_times[self.previous_state] = time_spent

            CLog.fsm(self, "TIMER", f"State '{self.previous_state}' took {time_spent:.2f} seconds")

        self.previous_state = new_state
        self.state_start_time = current_time

        if self.state_times:
            total_time = sum(self.state_times.values())
            CLog.fsm(self, "TIMER", f"Total time elapsed: {total_time:.2f} seconds")
            # Logger.info(self, f"State breakdown: {self.state_times}")

        CLog.fsm(
            self,
            "STATE",
            f"{self.previous_state} → {self.current_state}"
            if self.previous_state != self.current_state
            else self.current_state,
        )

        self.subtask_manager.hri.publish_display_step(new_state.lower(), GPSR_TASK_STEP_TOPIC)
        self._publish_command_index(self.executed_commands + len(self.batched_commands))

    def _publish_command_index(self, index: int) -> None:
        msg = Int32()
        msg.data = index
        self._command_index_pub.publish(msg)

    def _get_areas(self):
        """Areas table (poses) for distance estimation, fetched once per
        planning session.

        Prefers the live ``retrieve_areas`` service so poses reflect the
        current map; falls back to the static ``nav.areas_backup`` (loaded
        from areas.json) when the service is unavailable or returns junk.
        Returns ``{}`` when nothing usable is available, so the merger's
        locator degrades to unknown coordinates (LARGE_M) rather than raising.
        """
        if self._areas is not None:
            return self._areas
        areas = None
        try:
            status, data = self.subtask_manager.nav.retrieve_areas()
            if status == Status.EXECUTION_SUCCESS and isinstance(data, dict):
                areas = data
            else:
                self.get_logger().warning(
                    f"retrieve_areas returned status={status}; using backup map"
                )
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"retrieve_areas failed: {e}; using backup map")
        if areas is None:
            backup = getattr(self.subtask_manager.nav, "areas_backup", None)
            areas = backup if isinstance(backup, dict) else {}
        self._areas = areas
        return areas

    def _resolve_xy(self, location_name: str):
        """Resolve a free-text location to (x, y) using query_location +
        the live areas table (see _get_areas). Memoized per planning session."""
        if not location_name:
            return None
        if location_name in self._location_cache:
            return self._location_cache[location_name]
        xy = None
        try:
            hits = self.subtask_manager.hri.query_location(location_name, top_k=1)
            if hits:
                hit = hits[0]
                areas = self._get_areas()
                pose = areas.get(hit.area, {}).get(hit.subarea or "safe_place")
                if pose and len(pose) >= 2:
                    xy = (float(pose[0]), float(pose[1]))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"Location resolve failed for '{location_name}': {e}")
        self._location_cache[location_name] = xy
        return xy

    def _current_robot_xy(self):
        """Look up the robot's (x, y) in the map frame, or None on failure."""
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"current pose lookup failed: {e}")
            return None

    def _on_action_start(self, plan_action):
        kind = getattr(plan_action.action, "action", "?")
        self.subtask_manager.hri.publish_display_step(f"executing:{kind}", GPSR_TASK_STEP_TOPIC)

    def _on_action_complete(self, plan_action, status, result):
        if status == Status.EXECUTION_SUCCESS:
            self._completed.add((plan_action.source_cmd, plan_action.source_idx))
        try:
            self.subtask_manager.hri.add_command_history(plan_action.action, result, status)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"add_command_history failed: {e}")
        self.get_logger().info(
            f"action={getattr(plan_action.action, 'action', '?')}"
            f" cmd={plan_action.source_cmd} idx={plan_action.source_idx}"
            f" status={status}"
        )

    def _execute_interleaved_batch(self):
        """Plan + tick the py_trees tree for the current batch."""
        self._completed.clear()
        plan = merge(
            self.batched_commands,
            locator=self._resolve_xy,
            origin=self._current_robot_xy(),
        )
        self.get_logger().info(
            f"Merged {len(self.batched_commands)} commands → "
            f"{len(plan.actions)} interleaved actions"
        )
        spoken = self.subtask_manager.hri.parse_plan_to_text([pa.action for pa in plan.actions])
        self.subtask_manager.hri.say(f"I will now execute the merged plan: {spoken}")
        self.subtask_manager.hri.publish_display_step("executing", GPSR_TASK_STEP_TOPIC)

        fallback_lines = ["Falling back to the sequential plan."]
        for cmd_idx, per_cmd in enumerate(plan.fallback):
            if not per_cmd:
                continue
            cmd_text = self.subtask_manager.hri.parse_plan_to_text([pa.action for pa in per_cmd])
            fallback_lines.append(f"command {cmd_idx + 1}: {cmd_text}")
        fallback_text = "\n".join(fallback_lines)

        def _announce_fallback():
            try:
                self.subtask_manager.hri.publish_display_step("executing", GPSR_TASK_STEP_TOPIC)
                # Also log the full fallback text to the answers topic for visibility
                self.subtask_manager.hri.publish_display_step(fallback_text, ANSWER_PUBLISHER)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"publish_display_step failed: {e}")

        root = build_tree(
            plan,
            subtask_handlers=[self.gpsr_tasks, self.gpsr_individual_tasks],
            on_action_complete=self._on_action_complete,
            retry_count=2,
            global_budget_s=GLOBAL_BUDGET_S,
            on_fallback_entry=_announce_fallback,
            is_completed=lambda pa: (pa.source_cmd, pa.source_idx) in self._completed,
            on_action_start=self._on_action_start,
        )
        self.get_logger().info("Behaviour tree:\n" + render_tree_ascii(root))

        tree = py_trees.trees.BehaviourTree(root)
        try:
            tree.setup(timeout=15.0)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"tree setup raised: {e}")

        terminal = (
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
            py_trees.common.Status.INVALID,
        )
        while rclpy.ok():
            tree.tick()
            if root.status in terminal:
                break
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)

        self.get_logger().info(f"Batch tree finished with status={root.status}")
        self._location_cache.clear()
        self._areas = None

    def _execute_sequential_fallback(self, batched_commands):
        """Last-ditch path when py_trees is unavailable."""
        for cmd_list in batched_commands:
            for command in getattr(cmd_list, "commands", []):
                handler = search_command(
                    command.action, [self.gpsr_tasks, self.gpsr_individual_tasks]
                )
                if handler is None:
                    self.get_logger().error(f"No handler for action '{command.action}'")
                    continue
                try:
                    status, res = handler(command)
                    self.subtask_manager.hri.add_command_history(command, res, status)
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warning(f"Sequential fallback error on {command}: {e}")

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        self.subtask_manager.manipulation.move_to_position("nav_pose")

        if say:
            target = sublocation if sublocation else location
            pretty_target = target.replace("_", " ")
            CLog.nav(self, "MOVE", f"Moving to {target}")
            self.subtask_manager.hri.say(f"Now I will go to the {pretty_target}.", wait=False)

        result, error = self.subtask_manager.nav.move_to_location(location, sublocation)
        return result

    def timeout(self, timeout: int = 2):
        time.sleep(timeout)

    def run(self):
        """Finite State Machine"""

        if self.current_state == GPSRTM.TaskStates.WAITING_FOR_BUTTON:
            self.subtask_manager.hri.publish_display_step(
                "waiting_for_button", GPSR_TASK_STEP_TOPIC
            )
            self._publish_command_index(self.executed_commands)
            CLog.fsm(self, "STATE", "Waiting for start button...")
            self.subtask_manager.hri.reset_task_status()
            self.subtask_manager.hri.say("Waiting for start button to be pressed to start the task")

            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            CLog.fsm(
                self, "STATE", "Start button pressed, GPSR task will begin now.", level="success"
            )
            self.current_state = GPSRTM.TaskStates.START

        elif self.current_state == GPSRTM.TaskStates.START:
            self._track_state_change(GPSRTM.TaskStates.START)
            status = self.subtask_manager.nav.check_door()

            self.navigate_to("start_location", "", False)

            self.subtask_manager.hri.say(
                "Hi, my name is Frida and I am a general purpose robot. Please press the button on my screen to start telling me the commands one by one."
            )
            self.current_state = GPSRTM.TaskStates.WAIT_BUTTON_COMMAND

        elif self.current_state == GPSRTM.TaskStates.WAIT_BUTTON_COMMAND:
            self._track_state_change(GPSRTM.TaskStates.WAIT_BUTTON_COMMAND)
            if self.executed_commands >= MAX_COMMANDS:
                self.current_state = GPSRTM.TaskStates.DONE
                return
            say_time = 15
            start_time = time.time()
            self.subtask_manager.hri.reset_task_status()
            while not self.subtask_manager.hri.start_button_clicked:
                if time.time() - start_time > say_time:
                    start_time = time.time()
                    self.subtask_manager.hri.say(
                        "Please press the blue start button to begin.",
                        speed=1,
                    )
                rclpy.spin_once(self, timeout_sec=0.1)
            CLog.fsm(self, "STATE", "Start button pressed, hearing command now.", level="success")
            self.current_state = GPSRTM.TaskStates.WAITING_FOR_COMMAND

        elif self.current_state == GPSRTM.TaskStates.WAITING_FOR_COMMAND:
            self._track_state_change(GPSRTM.TaskStates.WAITING_FOR_COMMAND)
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_to_position("front_stare")

            if self.test_mode:
                if self.test_nl_commands:
                    user_command = self.test_nl_commands.pop(0)
                    s = Status.EXECUTION_SUCCESS
                    self.get_logger().info(f"TEST MODE: Using hardcoded command: {user_command}")
                else:
                    self.get_logger().info("TEST MODE: No more hardcoded commands. Waiting...")
                    time.sleep(1.0)
                    return
            else:
                s, user_command = self.subtask_manager.hri.ask_and_confirm(
                    "What is your command?",
                    "LLM_command",
                    context="The user was asked to say a command. We want to infer his complete instruction from the response",
                    confirm_question=confirm_command,
                    retries=ATTEMPT_LIMIT,
                    min_wait_between_retries=5.0,
                    skip_extract_data=True,
                    always_confirm=True,
                    max_audio_length=20.0,
                )

            if s != Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say("I am sorry, I could not understand you.")
                self.current_hear_attempt += 1
            elif not self.subtask_manager.hri.check_coherence(user_command):
                self.subtask_manager.hri.say(
                    "I didn't catch that correctly or the command was incomplete. Please tell me again.",
                    wait=True,
                )
                self.current_hear_attempt += 1
            else:
                self.subtask_manager.hri.say(
                    "I am planning how to perform your command, please wait a moment", wait=False
                )
                s, self.commands = self.subtask_manager.hri.command_interpreter(user_command)

                self.get_logger().info(
                    f"Interpreted command: {user_command} -> {str(self.commands)}"
                )

                if self.interleave_enabled:
                    self.batched_commands.append(CommandListLLM(commands=self.commands))
                    self._publish_command_index(self.executed_commands + len(self.batched_commands))
                    plan_text = self.subtask_manager.hri.parse_plan_to_text(self.commands)
                    self.subtask_manager.hri.say(plan_text)
                    if (
                        len(self.batched_commands) >= self.batch_size
                        or (self.executed_commands + len(self.batched_commands)) >= MAX_COMMANDS
                    ):
                        self.current_state = GPSRTM.TaskStates.PLAN_AND_EXECUTE_BATCH
                    else:
                        self.subtask_manager.hri.say(
                            "Please press the button to give me the next command.", wait=False
                        )
                        if self.test_mode:
                            self.current_state = GPSRTM.TaskStates.WAITING_FOR_COMMAND
                        else:
                            self.current_state = GPSRTM.TaskStates.WAIT_BUTTON_COMMAND
                else:
                    self.subtask_manager.hri.say("I will now execute your command.", wait=False)
                    plan_text = self.subtask_manager.hri.parse_plan_to_text(self.commands)
                    self.subtask_manager.hri.say(plan_text)
                    self.current_state = GPSRTM.TaskStates.EXECUTING_COMMAND

        elif self.current_state == GPSRTM.TaskStates.PLAN_AND_EXECUTE_BATCH:
            self._track_state_change(GPSRTM.TaskStates.PLAN_AND_EXECUTE_BATCH)
            batch_count = len(self.batched_commands)
            self._execute_interleaved_batch()
            self.executed_commands += batch_count
            self.batched_commands = []
            self.subtask_manager.hri.say(
                "I have finished executing your commands. I will return to the start position.",
                wait=False,
            )
            self.navigate_to("start_location", "", False)
            self.subtask_manager.manipulation.move_to_position("front_stare")
            self.current_state = GPSRTM.TaskStates.WAIT_BUTTON_COMMAND

        elif self.current_state == GPSRTM.TaskStates.EXECUTING_COMMAND:
            self.current_hear_attempt = 0
            if len(self.commands) == 0:
                self.current_state = GPSRTM.TaskStates.FINISHED_COMMAND
            else:
                command = self.commands.pop(0)

                self.subtask_manager.hri.publish_display_step(
                    f"executing:{command.action}", GPSR_TASK_STEP_TOPIC
                )
                self._publish_command_index(self.executed_commands)

                self.get_logger().info(f"Executing command: {str(command)}")
                # Neutral default per command: live camera; vision commands
                # switch to their own annotated feed when they start.
                self.subtask_manager.hri.publish_display_topic(IMAGE_ORIENTED_TOPIC)

                try:
                    exec_commad = search_command(
                        command.action,
                        [self.gpsr_tasks, self.gpsr_individual_tasks],
                    )
                    if exec_commad is None:
                        self.get_logger().error(
                            f"Command {command} is not implemented in GPSRTask or in the subtask managers."
                        )
                    else:
                        status, res = exec_commad(command)
                        self.get_logger().info(f"status-> {str(status)}")
                        self.get_logger().info(f"res-> {str(res)}")
                        self.subtask_manager.hri.add_command_history(
                            command,
                            res,
                            status,
                        )
                except Exception as e:
                    self.get_logger().warning(
                        f"Error occured while executing command ({str(command)}): " + str(e)
                    )

        elif self.current_state == GPSRTM.TaskStates.FINISHED_COMMAND:
            self._track_state_change(GPSRTM.TaskStates.FINISHED_COMMAND)
            self.subtask_manager.hri.say(
                "I have finished executing your command. I will return to the start position to await for new commands.",
                wait=False,
            )
            self.navigate_to("start_location", "", False)
            self.executed_commands += 1
            self.current_state = GPSRTM.TaskStates.WAIT_BUTTON_COMMAND
            self.subtask_manager.manipulation.move_to_position("front_stare")

        elif self.current_state == GPSRTM.TaskStates.DONE:
            self._track_state_change(GPSRTM.TaskStates.DONE)
            self.subtask_manager.hri.say(
                "I am done with the task. Hip hip, hooray!",
                wait=False,
            )
            self.subtask_manager.hri.reset_task_status()

            # Generate final timing report
            total_task_time = (datetime.now() - self.total_start_time).total_seconds()
            CLog.fsm(self, "TIMER", "=== FINAL TIMING REPORT ===")
            CLog.fsm(self, "TIMER", f"Total task time: {total_task_time:.2f} seconds")

            sorted_states = sorted(self.state_times.items(), key=lambda x: x[1], reverse=True)
            for state, time_spent in sorted_states:
                percentage = (time_spent / total_task_time) * 100 if total_task_time > 0 else 0
                CLog.fsm(self, "TIMER", f"{state}: {time_spent:.2f}s ({percentage:.1f}%)")

            CLog.fsm(self, "TIMER", "=== END TIMING REPORT ===")
            self.running_task = False


def main(args=None):
    rclpy.init(args=args)
    node = GPSRTM()

    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
