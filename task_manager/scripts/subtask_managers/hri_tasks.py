#!/usr/bin/env python3

"""
HRI Subtask manager
"""

import rclpy
from frida_constants.hri_constants import (
    ADD_ITEM_SERVICE,
    COMMAND_INTERPRETER_SERVICE,
    COMMON_INTEREST_SERVICE,
    EXTRACT_DATA_SERVICE,
    GRAMMAR_SERVICE,
    IS_POSITIVE_SERVICE,
    QUERY_ITEM_SERVICE,
    SPEAK_SERVICE,
    STT_SERVICE_NAME,
    WAKEWORD_TOPIC,
)
from frida_interfaces.srv import (
    STT,
    AddItem,
    CommandInterpreter,
    CommonInterest,
    ExtractInfo,
    Grammar,
    IsPositive,
    LLMWrapper,
    QueryItem,
    Speak,
)
from rclpy.node import Node
from std_msgs.msg import String
from utils.logger import Logger
from utils.task import Task

from subtask_managers.subtask_meta import SubtaskMeta

TIMEOUT = 5.0


class HRITasks(metaclass=SubtaskMeta):
    """Class to manage the vision tasks"""

    STATE = {"TERMINAL_ERROR": -1, "EXECUTION_ERROR": 0, "EXECUTION_SUCCESS": 1}

    # TODO: perform service checks using config.topic_config
    def __init__(self, task_manager, config=None, task=Task.RECEPTIONIST) -> None:
        self.node = task_manager
        self.keyword = ""
        self.speak_service = self.node.create_client(Speak, SPEAK_SERVICE)
        self.hear_service = self.node.create_client(STT, STT_SERVICE_NAME)
        self.extract_data_service = self.node.create_client(ExtractInfo, EXTRACT_DATA_SERVICE)

        self.command_interpreter_client = self.node.create_client(
            CommandInterpreter, COMMAND_INTERPRETER_SERVICE
        )
        self.task = task
        self.grammar_service = self.node.create_client(Grammar, GRAMMAR_SERVICE)
        self.common_interest_service = self.node.create_client(
            CommonInterest, COMMON_INTEREST_SERVICE
        )
        self.is_positive_service = self.node.create_client(IsPositive, IS_POSITIVE_SERVICE)

        self.query_item_client = self.node.create_client(QueryItem, QUERY_ITEM_SERVICE)
        self.add_item_client = self.node.create_client(AddItem, ADD_ITEM_SERVICE)
        self.llm_wrapper_service = self.node.create_client(LLMWrapper, "/nlp/llm")
        self.keyword_client = self.node.create_subscription(
            String, WAKEWORD_TOPIC, self._get_keyword, 10
        )

        self.services = {
            Task.RECEPTIONIST: {
                "hear": {
                    "client": self.hear_service,
                    "type": "service",
                },
                "say": {
                    "client": self.speak_service,
                    "type": "service",
                },
                "extract_data_service": {
                    "client": self.extract_data_service,
                    "type": "service",
                },
                "common_interest_service": {
                    "client": self.common_interest_service,
                    "type": "service",
                },
            },
        }

        self.setup_services()

    def setup_services(self):
        """Initialize services and actions"""

        if self.task not in self.services:
            Logger.error(self.node, "Task not available")
            return

        for key, service in self.services[self.task].items():
            if service["type"] == "service":
                if not service["client"].wait_for_service(timeout_sec=TIMEOUT):
                    Logger.warn(self.node, f"{key} service not initialized. ({self.task})")
            elif service["type"] == "action":
                if not service["client"].wait_for_server(timeout_sec=TIMEOUT):
                    Logger.warn(self.node, f"{key} action server not initialized. ({self.task})")

    def say(self, text: str, wait: bool = True) -> None:
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
        future = self.extract_data_service.call_async(request)
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
        try:
            data = eval(msg.data)
            self.keyword = data["keyword"]
        except Exception as e:
            self.node.get_logger().error(f"Error: {e}")
            self.keyword = ""

    def hear(self) -> str:
        self.node.get_logger().info("Hearing from user")
        request = STT.Request()

        future = self.hear_service.call_async(request)

        rclpy.spin_until_future_complete(self.node, future)

        return future.result().text_heard

    def interpret_keyword(self, keywords: list[str], timeout: float) -> str:
        start_time = self.node.get_clock().now()
        self.keyword = ""
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
        future = self.extract_data_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().answer

    def add_item(self, document: list, item_id: list, collection: str, metadata: list) -> str:
        """
        Adds new items to the ChromaDB collection.

        Args:
            document (list): List of documents to be added.
            item_id (list): List of item IDs corresponding to each document.
            collection (str): The collection to add the items to.
            metadata (list): List of metadata corresponding to each document.

        Returns:
            str: A message indicating the success or failure of the operation.
        """
        try:
            # Prepare the request with the necessary arguments
            request = AddItem.Request(
                document=document,  # List of documents
                id=item_id,  # List of item IDs
                collection=collection,  # The collection to add the items to
                metadata=metadata,  # Metadata as a JSON
            )

            # Make the service call
            future = self.add_item_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            # Check if the operation was successful
            if future.result().success:
                return "Items added successfully"
            else:
                return f"Failed to add items: {future.result().message}"

        except Exception as e:
            return f"Error: {str(e)}"

    def command_interpreter(self, text: str) -> CommandInterpreter.Response:
        request = CommandInterpreter.Request(text=text)
        future = self.command_interpreter_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result().commands

    def common_interest(self, person1, interest1, person2, interest2):
        request = CommonInterest.Request(
            person1=person1, interests1=interest1, person2=person2, interests2=interest2
        )
        future = self.common_interest_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().common_interest

    def is_positive(self, text):
        request = IsPositive.Request(text=text)
        future = self.is_positive_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().is_positive


if __name__ == "__main__":
    rclpy.init()
    node = Node("hri_tasks")
    vision_tasks = HRITasks(node)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
