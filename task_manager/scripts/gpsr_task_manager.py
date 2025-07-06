#!/usr/bin/env python3

"""
Task Manager for GPSR task of Robocup @Home 2025
"""

import time

import rclpy
from rclpy.node import Node
from subtask_managers.gpsr_single_tasks import GPSRSingleTask
from subtask_managers.gpsr_tasks import GPSRTask

from subtask_managers.gpsr_test_commands import get_gpsr_comands
from utils.baml_client.types import CommandListLLM
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3
MAX_COMMANDS = 3
USE_QR = False  # Set to False if you want to use speech recognition instead of QR code reading
QR_CODE_ATTEMPTS = 20


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

    class States:
        START = 0
        WAITING_FOR_COMMAND = 1
        EXECUTING_COMMAND = 2
        FINISHED_COMMAND = 3
        DONE = 4

    def __init__(self):
        """Initialize the node"""
        super().__init__("gpsr_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.GPSR, mock_areas=["navigation"])
        self.gpsr_tasks = GPSRTask(self.subtask_manager)
        self.gpsr_individual_tasks = GPSRSingleTask(self.subtask_manager)

        self.current_state = (
            # GPSRTM.States.START
            GPSRTM.States.EXECUTING_COMMAND
        )
        self.running_task = True
        self.current_attempt = 0
        self.executed_commands = 0
        # self.commands = get_gpsr_comands("takeObjFromPlcmt")
        self.commands = get_gpsr_comands("custom")
        # self.commands = []

        if isinstance(self.commands, dict):
            self.commands = CommandListLLM(**self.commands).commands

        Logger.info(self, "GPSRTMTaskManager has started.")

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

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
        future = self.subtask_manager.nav.move_to_location(location, sublocation)
        if "navigation" not in self.subtask_manager.get_mocked_areas():
            rclpy.spin_until_future_complete(self.subtask_manager.nav.node, future)

    def run(self):
        """State machine"""

        initial_state = self.current_state

        if self.current_state == GPSRTM.States.START:
            self.navigate_to("start_area", "", False)
            # res = "closed"
            # while res == "closed":
            #     time.sleep(1)
            #     status, res = self.subtask_manager.nav.check_door()
            #     if status == Status.EXECUTION_SUCCESS:
            #         Logger.info(self, f"Door status: {res}")
            #     else:
            #         Logger.error(self, "Failed to check door status")

            # self.subtask_manager.nav.move_to_location(location="start_area", sublocation="")  # LOCATION
            # self.subtask_manager.manipulation.move_joint_positions(
            #     named_position="front_stare", velocity=0.5, degrees=True
            # )
            self.subtask_manager.hri.say(
                "Hi, my name is Frida. I am a general purpose robot. I can help you with some tasks."
            )
            self.current_state = GPSRTM.States.WAITING_FOR_COMMAND
        elif self.current_state == GPSRTM.States.WAITING_FOR_COMMAND:
            if self.prev_state != self.current_state:
                self.navigate_to("start_area", "", False)

            if self.executed_commands >= MAX_COMMANDS:
                self.current_state = GPSRTM.States.DONE
                return

            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )

            if USE_QR:
                self.subtask_manager.hri.say(
                    "Please show me the QR code with your command. I will read it.", wait=False
                )
                for _ in range(QR_CODE_ATTEMPTS):
                    s, result = self.subtask_manager.vision.read_qr()
                    if s == Status.EXECUTION_SUCCESS and len(result) > 0:
                        self.subtask_manager.hri.say(
                            f"I have read the QR code. It says: {result}.", wait=False
                        )
                        user_command = result
                        break
                    self.timeout(0.3)
            else:
                s, user_command = self.subtask_manager.hri.ask_and_confirm(
                    "What is your command?",
                    "LLM_command",
                    context="The user was asked to say a command. We want to infer his complete instruction from the response",
                    confirm_question=confirm_command,
                    use_hotwords=False,
                    retries=ATTEMPT_LIMIT,
                    min_wait_between_retries=5.0,
                    skip_extract_data=True,
                )
            # gesture_person_list = ["waving person", "person raising their left arm", "person raising their right arm",
            #                "person pointing to the left", "person pointing to the right"]
            # pose_person_plural_list = ["sitting persons", "standing persons", "lying persons"]

            # s = Status.EXECUTION_SUCCESS
            # user_command = "go to the living room and count standing persons"
            # user_command = "Go to the kitchen table find Ale and tell her you"
            # user_command = "tell me how many standing persons are in the living room"

            if s != Status.EXECUTION_SUCCESS:
                if USE_QR:
                    self.subtask_manager.hri.say(
                        "I am sorry, I could not read the QR code. Please try again."
                    )
                else:
                    self.subtask_manager.hri.say("I am sorry, I could not understand you.")
                self.current_attempt += 1
            else:
                self.subtask_manager.hri.say(
                    "I am planning how to perform your command, please wait a moment", wait=False
                )
                s, self.commands = self.subtask_manager.hri.command_interpreter(user_command)

                self.get_logger().info(
                    f"Interpreted command: {user_command} -> {str(self.commands)}"
                )
                self.subtask_manager.hri.say("I will now execute your command", wait=False)
                self.current_state = GPSRTM.States.EXECUTING_COMMAND
        elif self.current_state == GPSRTM.States.EXECUTING_COMMAND:
            if len(self.commands) == 0:
                self.current_state = GPSRTM.States.FINISHED_COMMAND
            else:
                command = self.commands.pop(0)

                self.get_logger().info(f"Executing command: {str(command)}")

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

                        try:
                            status = status.value
                        except Exception:
                            try:
                                status = int(status)
                            except Exception:
                                pass

                        self.subtask_manager.hri.add_command_history(
                            command,
                            res,
                            status,
                        )
                except Exception as e:
                    self.get_logger().warning(
                        f"Error occured while executing command ({str(command)}): " + str(e)
                    )
            self.timeout(3)

        elif self.current_state == GPSRTM.States.FINISHED_COMMAND:
            self.subtask_manager.hri.say(
                "I have finished executing your command. I will return to the start position to await for new commands.",
                wait=False,
            )
            self.executed_commands += 1
            self.current_state = GPSRTM.States.WAITING_FOR_COMMAND
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )
        elif self.current_state == GPSRTM.States.DONE:
            self.subtask_manager.hri.say(
                "I am done with the task. I will now return to my home position.",
                wait=False,
            )
            self.running_task = False

        self.prev_state = initial_state


def main(args=None):
    """Main function"""
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
