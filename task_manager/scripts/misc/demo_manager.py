#!/usr/bin/env python3

"""
Task Manager for Demos
"""

import time

import rclpy
from rclpy.node import Node
from subtask_managers.gpsr_single_tasks import GPSRSingleTask
from subtask_managers.gpsr_tasks import GPSRTask

from utils.baml_client.types import CommandListLLM
from utils.logger import Logger
from utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3
MAX_COMMANDS = 3
USE_QR = True
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


class DemoTM(Node):
    """Class to manage the GPSR task"""

    class States:
        WAITING_FOR_BUTTON = 0
        WAITING_FOR_COMMAND = 1
        EXECUTING_COMMAND = 2

    def __init__(self):
        """Initialize the node"""
        super().__init__("demo_task_manager")
        self.subtask_manager = SubtaskManager(self, task=Task.DEMO, mock_areas=[""])
        self.gpsr_tasks = GPSRTask(self.subtask_manager)
        self.gpsr_individual_tasks = GPSRSingleTask(self.subtask_manager)

        self.current_state = DemoTM.States.WAITING_FOR_BUTTON

        self.prev_state = None
        self.running_task = True
        self.current_hear_attempt = 0
        self.executed_commands = 0
        self.commands = []

        if isinstance(self.commands, dict):
            self.commands = CommandListLLM(**self.commands).commands

        Logger.info(self, "DemoTMTaskManager has started.")

    def timeout(self, timeout: int = 2):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            pass

    def run(self):
        """State machine"""

        if self.current_state == DemoTM.States.WAITING_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.reset_task_status()
            self.subtask_manager.hri.say("Waiting for start button to be pressed to start the task")

            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, demo task will begin now")
            self.current_state = DemoTM.States.WAITING_FOR_COMMAND

        elif self.current_state == DemoTM.States.WAITING_FOR_COMMAND:
            self.subtask_manager.manipulation.move_joint_positions(
                named_position="front_stare", velocity=0.5, degrees=True
            )

            if self.executed_commands > 0:
                self.subtask_manager.hri.say(
                    "I'll wait for someone to say Frida to execute another command."
                )
            else:
                self.subtask_manager.hri.say(
                    "I'll wait for someone to say Frida to execute a command."
                )

            self.subtask_manager.hri.keyword = ""

            while self.subtask_manager.hri.keyword != "frida":
                rclpy.spin_once(self, timeout_sec=0.1)

            self.subtask_manager.hri.say(
                "Hello, I heard someone said Frida, and I am ready to assist you. By the way, currently I can only answer questions or pick and place an object in front of me."
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
            )

            if "pick" in user_command.lower():
                self.subtask_manager.hri.say("I will pick the object in front of me.")
                s, target_info = self.subtask_manager.hri.extract_data(
                    "LLM_object", user_command, "the object that the user want to pick"
                )
                self.gpsr_individual_tasks.pick_object(
                    {"action": "pick_object", "object_to_pick": target_info},
                )
            elif "place" in user_command.lower():
                self.subtask_manager.hri.say("I will place the object in front of me.")
                self.gpsr_individual_tasks.place_object(
                    {"action": "place_object"},
                )
            else:
                self.subtask_manager.hri.say(
                    "I'm thinking how I can answer your question, please wait a moment while I process it.",
                    wait=False,
                )

                self.gpsr_individual_tasks.say_with_context(
                    {
                        "action": "say_with_context",
                        "user_instruction": "ignore",
                        "previous_command_info": [user_command],
                    }
                )

            self.executed_commands += 1

            self.subtask_manager.hri.say("I've finished executing the command.")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = DemoTM()

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
