#!/usr/bin/env python3

"""
Task Manager for GPSR task of Robocup @Home 2025
"""

import rclpy
from rclpy.node import Node
from subtask_managers.gpsr_single_tasks import GPSRSingleTask
from subtask_managers.gpsr_tasks import GPSRTask
from utils.logger import Logger
from utils.status import Status
from utils.subtask_manager import SubtaskManager, Task

# from subtask_managers.gpsr_test_commands import get_gpsr_comands


ATTEMPT_LIMIT = 3
MAX_COMMANDS = 3
START = "START"


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

        self.current_state = GPSRTM.States.EXECUTING_COMMAND
        self.running_task = True
        self.current_attempt = 0
        self.executed_commands = 0
        # self.commands = get_gpsr_comands("takeObjFromPlcmt")
        self.commands = [
            # {"action": "go", "complement": "kitchen table", "characteristic": ""},
            {"action": "visual_info", "complement": "heaviest", "characteristic": "food"},
            # {"action": "go", "complement": "start_location", "characteristic": ""},
            {
                "action": "contextual_say",
                "complement": "tell me what is the heaviest object in the kitchen",
                "characteristic": "visual_info",
            },
        ]

        Logger.info(self, "GPSRTMTaskManager has started.")

    def run(self):
        """State machine"""

        if self.current_state == GPSRTM.States.START:
            self.subtask_manager.hri.say(
                "Hi, my name is Frida. I am a general purpose robot. I can help you with some tasks."
            )
            self.current_state = GPSRTM.States.WAITING_FOR_COMMAND
        elif self.current_state == GPSRTM.States.WAITING_FOR_COMMAND:
            if self.executed_commands >= MAX_COMMANDS:
                self.current_state = GPSRTM.States.DONE
                return

            s, user_command = self.subtask_manager.hri.ask_and_confirm(
                "What is your command?",
                "command",
                context="The user was asked to say a command. We want to infer his command from the response",
                confirm_question=confirm_command,
                use_hotwords=False,
                retries=ATTEMPT_LIMIT,
                min_wait_between_retries=5.0,
            )
            if s != Status.EXECUTION_SUCCESS:
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
                self.subtask_manager.hri.say("I will now execute your command")
                self.current_state = GPSRTM.States.EXECUTING_COMMAND
        elif self.current_state == GPSRTM.States.EXECUTING_COMMAND:
            if len(self.commands) == 0:
                self.current_state = GPSRTM.States.FINISHED_COMMAND
            else:
                command = self.commands.pop(0)
                exec_commad = search_command(
                    command["action"],
                    [self.gpsr_tasks, self.gpsr_individual_tasks],
                )
                if exec_commad is None:
                    self.get_logger().error(
                        f"Command {command} is not implemented in GPSRTask or in the subtask managers."
                    )
                else:
                    Logger.info(self, f"Executing command: {command}")
                    # self.subtask_manager.hri.say(f"Executing command: {command}")
                    status, res = exec_commad(command["complement"], command["characteristic"])
                    self.subtask_manager.hri.add_command_history(
                        command["action"],
                        command["complement"],
                        command["characteristic"],
                        res,
                        status.value,
                    )
        elif self.current_state == GPSRTM.States.FINISHED_COMMAND:
            self.subtask_manager.hri.say(
                "I have finished executing your command. I will return to the start position to await for new commands.",
                wait=False,
            )
            self.executed_commands += 1
            self.current_state = GPSRTM.States.WAITING_FOR_COMMAND
        elif self.current_state == GPSRTM.States.DONE:
            self.subtask_manager.hri.say(
                "I am done with the task. I will now return to my home position.",
                wait=False,
            )
            self.running_task = False


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
