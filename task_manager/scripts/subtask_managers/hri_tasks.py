#!/usr/bin/env python3

"""
HRI Subtask manager
"""

import rclpy
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
    LLMWrapper,
    QueryItem,
    Speak,
)
from rclpy.node import Node
from std_msgs.msg import String

from subtask_managers.subtask_meta import SubtaskMeta

TIMEOUT = 5.0


class HRITasks(metaclass=SubtaskMeta):
    """Class to manage the vision tasks"""

    STATE = {"TERMINAL_ERROR": -1, "EXECUTION_ERROR": 0, "EXECUTION_SUCCESS": 1}

    # TODO: perform service checks using config.topic_config
    def __init__(self, task_manager, config=None) -> None:
        self.node = task_manager
        self.keyword = ""
        self.speak_service = self.node.create_client(Speak, SPEAK_SERVICE)
        self.hear_service = self.node.create_client(STT, STT_SERVICE_NAME)
        self.extract_data_service = self.node.create_client(ExtractInfo, EXTRACT_DATA_SERVICE)

        self.command_interpreter_client = self.node.create_client(
            CommandInterpreter, COMMAND_INTERPRETER_SERVICE
        )
        self.grammar_service = self.node.create_client(Grammar, GRAMMAR_SERVICE)

        self.query_item_client = self.node.create_client(QueryItem, QUERY_ITEM_SERVICE)
        self.add_item_client = self.node.create_client(AddItem, ADD_ITEM_SERVICE)
        self.llm_wrapper_service = self.node.create_client(LLMWrapper, "/nlp/llm")
        self.keyword_client = self.node.create_subscription(
            String, "/wakeword_detected", self._get_keyword, 10
        )

    def say(self, text: str, wait: bool = False) -> None:
        """Method to publish directly text to the speech node"""
        self.node.get_logger().info(f"Sending to saying service: {text}")
        request = Speak.Request(text=text)

        future = self.speak_service.call_async(request)

        if wait:
            self.node.get_logger().info("in wait")
            rclpy.spin_until_future_complete(self.node, future)
            self.node.get_logger().info("after future complete")
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

    def execute_command(self, command: str, complement: str, characteristic: str) -> None:
        if command == "speak":
            return self.say(complement)
        elif command == "clarification":
            self.say("Sorry, I don't undestand your command.")
            self.say(command.complement)

        else:
            return self.say(f"Sorry, I don't know how to {command}")

    def _get_keyword(self, msg: String) -> None:
        data = eval(msg.data)
        self.keyword = data["keyword"]

    def hear(self) -> str:
        self.node.get_logger().info("Hearing from user")
        request = STT.Request()

        future = self.hear_service.call_async(request)

        rclpy.spin_until_future_complete(self.node, future)

        return future.result().text_heard

    def interpret_keyword(self, keywords: list[str], timeout: float) -> str:
        start_time = self.node.get_clock().now()
        self.keyword = "None"
        while (
            self.keyword not in keywords
            and ((self.node.get_clock().now() - start_time).nanoseconds / 1e9) < timeout
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        return self.keyword

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
        request = QueryItem.Request(query=query, collection=collection, topk=top_k)
        future = self.query_item_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result().results

    def ask(self, question: str) -> str:
        self.llm_wrapper_service
        request = LLMWrapper.Request(question=question)
        future = self.extract_data_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().answer

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
