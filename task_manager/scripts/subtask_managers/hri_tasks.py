#!/usr/bin/env python3

"""
HRI Subtask manager
"""

from typing import Union

import rclpy
from rclpy.node import Node
from subtask_managers.subtask_meta import SubtaskMeta

from frida_constants.hri_constants import (
    COMMAND_INTERPRETER_SERVICE,
    EXTRACT_DATA_SERVICE,
    GRAMMAR_SERVICE,
    HEAR_SERVICE,
    SPEAK_SERVICE,
)
from frida_interfaces.srv import STT, CommandInterpreter, ExtractInfo, Grammar, Speak

TIMEOUT = 5.0


class HRITasks(metaclass=SubtaskMeta):
    """Class to manage the vision tasks"""

    STATE = {"TERMINAL_ERROR": -1, "EXECUTION_ERROR": 0, "EXECUTION_SUCCESS": 1}

    # TODO: perform service checks using config.topic_config
    def __init__(self, task_manager, config=None) -> None:
        self.node = task_manager

        self.speak_client = self.node.create_client(Speak, SPEAK_SERVICE)
        self.hear_client = self.node.create_client(STT, HEAR_SERVICE)
        self.extract_data_client = self.node.create_client(
            ExtractInfo, EXTRACT_DATA_SERVICE
        )

        self.command_interpreter_client = self.node.create_client(
            CommandInterpreter, COMMAND_INTERPRETER_SERVICE
        )
        self.grammar_service = self.node.create_client(Grammar, GRAMMAR_SERVICE)

    def say(self, text: str, wait: bool = False) -> None:
        """Method to publish directly text to the speech node"""
        self.node.get_logger().info(f"Sending to saying service: {text}")
        request = Speak.Request(text=text)

        future = self.speak_client.call_async(request)

        if wait:
            rclpy.spin_until_future_complete(self.node, future)
            return (
                HRITasks.STATE["EXECUTION_SUCCESS"]
                if future.result().success
                else HRITasks.STATE["EXECUTION_ERROR"]
            )
        return HRITasks.STATE["EXECUTION_SUCCESS"]

    def extract_data(self, query, complete_text) -> str:
        request = ExtractInfo.Request(data=query, full_text=complete_text)
        future = self.extract_data_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().result

    def hear(self, timeout: float = 15.0) -> str:
        request = STT.Request()

        future = self.hear_client.call_async(request)

        rclpy.spin_until_future_complete(self.node, future)

        return future.result().text_heard

    # TODO
    def interpret_keyword(self, keyword: str, timeout: float) -> str:
        pass

    def refactor_text(self, text: str) -> str:
        request = Grammar.Request(text=text)
        future = self.grammar_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().corrected_text

    # TODO
    def find_closest(self, query: str, options: Union[list[str], str]) -> str:
        pass

    # TODO
    def ask(self, question: str) -> str:
        pass

    def command_interpreter(self, text: str) -> CommandInterpreter.Response:
        request = CommandInterpreter.Request(text=text)
        future = self.command_interpreter_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result().commands


if __name__ == "__main__":
    rclpy.init()
    node = Node("hri_tasks")
    vision_tasks = HRITasks(node)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
