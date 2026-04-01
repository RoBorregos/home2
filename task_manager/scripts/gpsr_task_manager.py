#!/usr/bin/env python3

"""
Task Manager for GPSR task of Robocup @Home 2026
"""

import time
from datetime import datetime

import rclpy
from frida_constants.vision_constants import IMAGE_TOPIC_HRIC
from rclpy.node import Node

from task_manager.subtask_managers.gpsr_single_tasks import GPSRSingleTask
from task_manager.subtask_managers.gpsr_tasks import GPSRTask

# from subtask_managers.gpsr_test_commands import get_gpsr_comands
from task_manager.utils.baml_client.types import CommandListLLM
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3
MAX_COMMANDS = 3


def confirm_command(interpreted_text, target_info):
    return f"Did you say {target_info}?"


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
        DONE = "DONE"
        WAIT_BUTTON_COMMAND = "WAIT_BUTTON_COMMAND"

    def __init__(self):
        """Initialize the node"""
        super().__init__("gpsr_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.GPSR, mock_areas=[""])
        self.gpsr_tasks = GPSRTask(self.subtask_manager)
        self.gpsr_individual_tasks = GPSRSingleTask(self.subtask_manager)

        self.current_state = (
            GPSRTM.TaskStates.START
            # GPSRTM.TaskStates.WAITING_FOR_BUTTON
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

        # State timing variables
        self.state_start_time = None
        self.state_times = {}
        self.total_start_time = datetime.now()
        self.previous_state = None

        Logger.info(self, "GPSRTMTaskManager has started.")

    def _track_state_change(self, new_state: str):
        """Track state changes and time spent in each state"""
        current_time = datetime.now()

        if self.previous_state and self.state_start_time:
            time_spent = (current_time - self.state_start_time).total_seconds()
            if self.previous_state in self.state_times:
                self.state_times[self.previous_state] += time_spent
            else:
                self.state_times[self.previous_state] = time_spent

            Logger.info(self, f"State '{self.previous_state}' took {time_spent:.2f} seconds")

        self.previous_state = new_state
        self.state_start_time = current_time

        if self.state_times:
            total_time = sum(self.state_times.values())
            Logger.info(self, f"Total time elapsed: {total_time:.2f} seconds")
            Logger.info(self, f"State breakdown: {self.state_times}")

        Logger.state(self, self.current_state)

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to the location"""
        if say:
            self.subtask_manager.hri.say(
                f"I will now guide you to the {location}. Please follow me."
            )
            self.subtask_manager.manipulation.follow_face(False)

        self.subtask_manager.manipulation.move_joint_positions(
            named_position="nav_pose", velocity=0.5, degrees=True
        )
        self.subtask_manager.nav.resume_nav()
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)

        self.subtask_manager.nav.pause_nav()

    def timeout(self, timeout: int = 2):
        time.sleep(timeout)

    def run(self):
        """Finite State Machine"""

        if self.current_state == GPSRTM.TaskStates.WAITING_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.reset_task_status()
            self.subtask_manager.hri.say("Waiting for start button to be pressed to start the task")

            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, receptionist task will begin now")
            self.current_state = GPSRTM.TaskStates.START

        elif self.current_state == GPSRTM.TaskStates.START:
            self._track_state_change(GPSRTM.TaskStates.START)
            res = "closed"
            while res == "closed":
                time.sleep(1)
                status, res = self.subtask_manager.nav.check_door()
                if status == Status.EXECUTION_SUCCESS:
                    Logger.info(self, f"Door status: {res}")
                else:
                    Logger.error(self, "Failed to check door status")

            self.navigate_to("start_area", "", False)

            self.subtask_manager.hri.say(
                "Hi, my name is Frida. I am a general purpose robot. I can help you with some tasks. Please tell them to me one by one."
            )
            self.current_state = GPSRTM.TaskStates.WAIT_BUTTON_COMMAND

        elif self.current_state == GPSRTM.TaskStates.WAIT_BUTTON_COMMAND:
            self._track_state_change(GPSRTM.TaskStates.WAIT_BUTTON_COMMAND)
            if self.executed_commands >= MAX_COMMANDS:
                self.current_state = GPSRTM.TaskStates.DONE
                return
            say_time = 5
            start_time = time.time()
            self.subtask_manager.hri.reset_task_status()
            while not self.subtask_manager.hri.start_button_clicked:
                if time.time() - start_time > say_time:
                    start_time = time.time()
                    self.subtask_manager.hri.say(
                        "Waiting for the blue start button on my screen to be pressed to hear the command.",
                        speed=1,
                    )
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, receptionist task will begin now")
            self.current_state = GPSRTM.TaskStates.WAITING_FOR_COMMAND

        elif self.current_state == GPSRTM.TaskStates.WAITING_FOR_COMMAND:
            self._track_state_change(GPSRTM.TaskStates.WAITING_FOR_COMMAND)
            self.subtask_manager.manipulation.follow_face(False)
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )

            s, user_command = self.subtask_manager.hri.ask_and_confirm(
                "What is your command?",
                "LLM_command",
                context="The user was asked to say a command. We want to infer his complete instruction from the response",
                confirm_question=confirm_command,
                use_hotwords=False,
                retries=ATTEMPT_LIMIT,
                min_wait_between_retries=5.0,
                skip_extract_data=True,
                always_confirm=True,
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
                self.subtask_manager.hri.say(
                    "I will now execute your command. My plan is " + str(str(self.commands)),
                    wait=False,
                )
                self.current_state = GPSRTM.TaskStates.EXECUTING_COMMAND

        elif self.current_state == GPSRTM.TaskStates.EXECUTING_COMMAND:
            self._track_state_change(GPSRTM.TaskStates.EXECUTING_COMMAND)
            self.current_hear_attempt = 0
            if len(self.commands) == 0:
                self.current_state = GPSRTM.TaskStates.FINISHED_COMMAND
            else:
                command = self.commands.pop(0)

                self.get_logger().info(f"Executing command: {str(command)}")
                self.subtask_manager.hri.publish_display_topic(IMAGE_TOPIC_HRIC)

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
            self.navigate_to("start_area", "", False)
            self.executed_commands += 1
            self.current_state = GPSRTM.TaskStates.WAIT_BUTTON_COMMAND
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )

        elif self.current_state == GPSRTM.TaskStates.DONE:
            self._track_state_change(GPSRTM.TaskStates.DONE)
            self.subtask_manager.hri.say(
                "I am done with the task. Hip hip, hooray!",
                wait=False,
            )
            self.subtask_manager.hri.reset_task_status()

            # Generate final timing report
            total_task_time = (datetime.now() - self.total_start_time).total_seconds()
            Logger.info(self, "=== FINAL TIMING REPORT ===")
            Logger.info(self, f"Total task time: {total_task_time:.2f} seconds")

            sorted_states = sorted(self.state_times.items(), key=lambda x: x[1], reverse=True)
            for state, time_spent in sorted_states:
                percentage = (time_spent / total_task_time) * 100
                Logger.info(self, f"{state}: {time_spent:.2f}s ({percentage:.1f}%)")

            Logger.info(self, "=== END TIMING REPORT ===")
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
