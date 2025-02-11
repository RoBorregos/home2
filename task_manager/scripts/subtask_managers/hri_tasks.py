#!/usr/bin/env python3

"""
HRI Subtask manager
"""

from typing import Union

import rclpy
from rclpy.node import Node
from subtask_meta import SubtaskMeta

from frida_constants.hri_constants import (
    COMMAND_INTERPRETER_SERVICE,
    DATA_EXTRACTOR_SERVICE,
    HEAR_SERVICE,
    SPEAK_SERVICE,
)
from frida_interfaces.srv import STT, CommandInterpreter, ExtractInfo, Speak

TIMEOUT = 5.0


class HRITasks(metaclass=SubtaskMeta):
    """Class to manage the vision tasks"""

    STATE = {"TERMINAL_ERROR": -1, "EXECUTION_ERROR": 0, "EXECUTION_SUCCESS": 1}

    def __init__(self, task_manager, config) -> None:
        self.node = task_manager

        self.speak_service = self.node.create_client(Speak, SPEAK_SERVICE)
        self.hear_service = self.node.create_client(STT, HEAR_SERVICE)
        self.extract_data_service = self.node.create_client(ExtractInfo, DATA_EXTRACTOR_SERVICE)

        self.command_interpreter_service = self.node.create_client(
            CommandInterpreter, COMMAND_INTERPRETER_SERVICE
        )

    def say(self, text: str, now: bool = False) -> None:
        """Method to publish directly text to the speech node"""
        self.node.get_logger().info(f"Sending to saying service: {text}")

        self.speak_service(text)

    def extract_date(self, query, complete_text) -> str:
        pass

    def hear(self, timeout: float) -> str:
        pass

    def interpret_keyword(self, keyword: str, timeout: float) -> str:
        pass

    def refactor_sentence(self, sentence: str) -> str:
        pass

    def find_closest(self, query: str, options: Union[list[str], str]) -> str:
        pass

    def ask(self, question: str) -> str:
        """Method to publish directly text to the speech node"""
        pass

    def command_interpreter(self, text: str) -> str:
        pass


if __name__ == "__main__":
    rclpy.init()
    node = Node("hri_tasks")
    vision_tasks = HRITasks(node)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
