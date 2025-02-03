#!/usr/bin/env python3

"""
HRI Subtask manager
"""

from typing import Union

import rclpy
from rclpy.node import Node
from subtask_managers.subtask_meta import SubtaskMeta

from frida_constants.hri_constants import (
    ADD_ITEM_SERVICE,
    COMMAND_INTERPRETER_SERVICE,
    EXTRACT_DATA_SERVICE,
    GRAMMAR_SERVICE,
    QUERY_ITEM_SERVICE,
    SPEAK_SERVICE,
    STT_SERVICE_NAME,
)
from frida_interfaces.srv import (
    STT,
    AddItem,
    CommandInterpreter,
    ExtractInfo,
    Grammar,
    QueryItem,
    Speak,
)

TIMEOUT = 5.0


class HRITasks(metaclass=SubtaskMeta):
    """Class to manage the vision tasks"""

    STATE = {"TERMINAL_ERROR": -1, "EXECUTION_ERROR": 0, "EXECUTION_SUCCESS": 1}

    # TODO: perform service checks using config.topic_config
    def __init__(self, task_manager, config=None) -> None:
        self.node = task_manager

        self.speak_client = self.node.create_client(Speak, SPEAK_SERVICE)
        self.hear_client = self.node.create_client(STT, STT_SERVICE_NAME)
        self.extract_data_client = self.node.create_client(
            ExtractInfo, EXTRACT_DATA_SERVICE
        )

        self.command_interpreter_client = self.node.create_client(
            CommandInterpreter, COMMAND_INTERPRETER_SERVICE
        )
        self.grammar_service = self.node.create_client(Grammar, GRAMMAR_SERVICE)

        self.query_item_client = self.node.create_client(QueryItem, QUERY_ITEM_SERVICE)
        self.add_item_client = self.node.create_client(AddItem, ADD_ITEM_SERVICE)

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
        """
        Extracts data from the given query and complete text.

        Args:
            query (str): specifies what to extract from complete_text.
            complete_text (str): The complete text from which data is to be extracted.

        Returns:
            str: The extracted data as a string. If no data is found, an empty string is returned.
        """
        self.node.get_logger().info(
            f"Sending to extract data service: query={query}, text={complete_text}"
        )

        request = ExtractInfo.Request(data=query, full_text=complete_text)
        future = self.extract_data_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().result

    def hear(self) -> str:
        request = STT.Request()

        future = self.hear_client.call_async(request)

        rclpy.spin_until_future_complete(self.node, future)

        return future.result().text_heard

    # TODO
    def interpret_keyword(self, keyword: Union[list[str], str], timeout: float) -> str:
        """
        Interprets the given keyword(s) within a specified timeout period.
        Args:
            keyword (Union[list[str], str]): The keyword or list of keywords to interpret.
            timeout (float): The maximum time allowed for interpretation in seconds.
        Returns:
            str: The interpreted result as a string, or an empty string if no result is found within the timeout period.
        """
        pass

    def refactor_text(self, text: str) -> str:
        request = Grammar.Request(text=text)
        future = self.grammar_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().corrected_text

    def find_closest(self, query: str, collection: str, top_k: int = 1) -> list[str]:
        """
        Finds the closest matching item in a specified collection based on the given query.

        Args:
            query (str): The search query to find the closest match for.
            collection (str): The name of the collection to search within.
            top_k (int, optional): The number of top matches to return. Defaults to 1.

        Returns:
            list[str]: The closest matching item(s) from the collection.
        """
        request = QueryItem.Request(query=query, collection=collection, top_k=top_k)
        future = self.grammar_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result().results

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
