#!/usr/bin/env python3

"""
HRI Subtask manager
"""

import re
from typing import Union

import rclpy
from frida_constants.hri_constants import (
    ADD_ENTRY_SERVICE,
    COMMAND_INTERPRETER_SERVICE,
    COMMON_INTEREST_SERVICE,
    EXTRACT_DATA_SERVICE,
    GRAMMAR_SERVICE,
    IS_NEGATIVE_SERVICE,
    IS_POSITIVE_SERVICE,
    LLM_WRAPPER_SERVICE,
    QUERY_ENTRY_SERVICE,
    SPEAK_SERVICE,
    STT_SERVICE_NAME,
    USEFUL_AUDIO_NODE_NAME,
    WAKEWORD_TOPIC,
    CATEGORIZE_SERVICE,
)
from frida_interfaces.srv import (
    STT,
    AddEntry,
    CommandInterpreter,
    CommonInterest,
    ExtractInfo,
    Grammar,
    IsNegative,
    IsPositive,
    LLMWrapper,
    QueryEntry,
    Speak,
    CategorizeShelves,
)

from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import String
from utils.decorators import service_check
from utils.logger import Logger
from utils.status import Status
from utils.task import Task

from subtask_managers.subtask_meta import SubtaskMeta

TIMEOUT = 5.0


def confirm_query(interpreted_text, target_info):
    return f"Did you say {target_info}?"


def confirm_interpretation(interpreted_text, target_info):
    return f"Did you say {interpreted_text}?"


class HRITasks(metaclass=SubtaskMeta):
    """Class to manage the vision tasks"""

    def __init__(self, task_manager: Node, config=None, task=Task.RECEPTIONIST) -> None:
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
        self.is_negative_service = self.node.create_client(IsNegative, IS_NEGATIVE_SERVICE)

        self.query_item_client = self.node.create_client(QueryEntry, QUERY_ENTRY_SERVICE)
        self.add_item_client = self.node.create_client(AddEntry, ADD_ENTRY_SERVICE)
        self.llm_wrapper_service = self.node.create_client(LLMWrapper, LLM_WRAPPER_SERVICE)
        self.categorize_service = self.node.create_client(CategorizeShelves, CATEGORIZE_SERVICE)
        self.keyword_client = self.node.create_subscription(
            String, WAKEWORD_TOPIC, self._get_keyword, 10
        )

        self.useful_audio_params = self.node.create_client(
            SetParameters, f"/{USEFUL_AUDIO_NODE_NAME}/set_parameters"
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

    @service_check("speak_service", Status.SERVICE_CHECK, TIMEOUT)
    def say(self, text: str, wait: bool = True) -> None:
        """Method to publish directly text to the speech node"""
        self.node.get_logger().info(f"Sending to saying service: {text}")
        request = Speak.Request(text=text)

        future = self.speak_service.call_async(request)

        if wait:
            rclpy.spin_until_future_complete(self.node, future)
            return Status.EXECUTION_SUCCESS if future.result().success else Status.EXECUTION_ERROR
        return Status.EXECUTION_SUCCESS

    @service_check("extract_data_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def extract_data(self, query, complete_text, context="") -> str:
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

        request = ExtractInfo.Request(data=query, full_text=complete_text, context=context)
        future = self.extract_data_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        execution_status = (
            Status.EXECUTION_SUCCESS if len(future.result().result) > 0 else Status.TARGET_NOT_FOUND
        )

        return execution_status, future.result().result

    def execute_command(self, command: str, complement: str, characteristic: str) -> None:
        if command == "speak":
            self.say(complement)
            return Status.EXECUTION_SUCCESS
        elif command == "clarification":
            self.say("Sorry, I don't undestand your command.")
            self.say(command.complement)
            return Status.EXECUTION_SUCCESS
        else:
            self.say(f"Sorry, I don't know how to {command}")
            return Status.TARGET_NOT_FOUND

    def _get_keyword(self, msg: String) -> None:
        try:
            data = eval(msg.data)
            self.keyword = data["keyword"]
        except Exception as e:
            self.node.get_logger().error(f"Error: {e}")
            self.keyword = ""

    @service_check("hear_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def hear(self, min_audio_length=0.5, max_audio_length=10.0) -> str:
        if min_audio_length > 0:
            self.set_double_param("MIN_AUDIO_DURATION", float(min_audio_length))

        if max_audio_length > 0:
            self.set_double_param("MAX_AUDIO_DURATION", float(max_audio_length))

        request = STT.Request()

        future = self.hear_service.call_async(request)
        self.node.get_logger().info("Hearing from the user...")
        rclpy.spin_until_future_complete(self.node, future)

        execution_status = (
            Status.EXECUTION_SUCCESS
            if len(future.result().text_heard) > 0
            else Status.TARGET_NOT_FOUND
        )

        return execution_status, future.result().text_heard

    def confirm(
        self,
        question: str,
        use_hotwords: bool = True,
        retries: int = 3,
        wait_between_retries: float = 5,
    ):
        """
        Method to confirm a specific question. Could be used for deus ex machina, to confirm a specific action.

        Args:
            question: the inquiry to confirm
            use_hotwords: if True, the robot will only react if 'yes' or 'no' is mentioned. Otherwise, it will hear any type of answer and interpret it with an llm.
            retries: the amount of times to try before returning false
            wait_between_retries: the amount of time to wait between retries
        Returns:
            Status: the status of the execution
            str: "yes" (user confirms), "no" (user doesn't confirm), or "" (no response interpreted).
        """
        current_attempt = 0
        while current_attempt < retries:
            current_attempt += 1

            # Say the question
            self.say(question)

            if use_hotwords:
                self.say("Please confirm by saying yes or no")

                s, keyword = self.interpret_keyword(["yes", "no"], timeout=wait_between_retries)
                if s == Status.EXECUTION_SUCCESS:
                    return Status.EXECUTION_SUCCESS, keyword
            else:
                start_time = self.node.get_clock().now()
                while (
                    (self.node.get_clock().now() - start_time).nanoseconds / 1e9
                ) < wait_between_retries:
                    s, interpret_text = self.hear()
                    if s == Status.EXECUTION_SUCCESS:
                        if self.is_positive(interpret_text)[1]:
                            return Status.EXECUTION_SUCCESS, "yes"
                        elif self.is_negative(interpret_text)[1]:
                            return Status.EXECUTION_SUCCESS, "no"

        return Status.TIMEOUT, ""

    def ask_and_confirm(
        self,
        question: str,
        query: str,
        context: str = "",
        confirm_question: Union[str, callable] = confirm_query,
        use_hotwords: bool = True,
        retries: int = 3,
        min_wait_between_retries: float = 5,
    ):
        """
        Method to confirm a specific question.

        Args:
            question: the inquiry to ask
            query: the data to extract from the interpreted text
            context: the context of the question. It could be used to help the extraction.
            confirm_question: a string or a callable function that returns a string used confirm the answer
            use_hotwords: if True, the robot will only react if 'yes' or 'no' is the confirmations. Otherwise, it will hear any type of answer and interpret it with an llm.
            retries: the amount of times to try before returning false
            min_wait_between_retries: the minimum amount of time to wait between retries

        Returns:
            Status: the status of the execution
            str: answer to the question
        """
        current_attempt = 0
        while current_attempt < retries:
            current_attempt += 1

            start_time = self.node.get_clock().now()

            self.say(question)
            s, interpreted_text = self.hear()
            print(f"Interpreted text: {interpreted_text}")

            if s == Status.EXECUTION_SUCCESS:
                s, target_info = self.extract_data(query, interpreted_text, context)

                if s == Status.TARGET_NOT_FOUND:
                    target_info = interpreted_text

                # Determine the confirmation question
                if callable(confirm_question):
                    confirmation_text = confirm_question(interpreted_text, target_info)
                else:
                    confirmation_text = confirm_question

                s, confirmation = self.confirm(confirmation_text, use_hotwords, 1)

                if confirmation == "yes":
                    return Status.EXECUTION_SUCCESS, target_info

            # Wait for the minimum time between retries
            while (
                (self.node.get_clock().now() - start_time).nanoseconds / 1e9
            ) < min_wait_between_retries:
                rclpy.spin_once(self.node, timeout_sec=0.1)

        return Status.TIMEOUT, ""

    def interpret_keyword(self, keywords: list[str], timeout: float) -> str:
        start_time = self.node.get_clock().now()
        self.keyword = ""
        while (
            self.keyword not in keywords
            and ((self.node.get_clock().now() - start_time).nanoseconds / 1e9) < timeout
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        execution_status = (
            Status.EXECUTION_SUCCESS if self.keyword in keywords else Status.TARGET_NOT_FOUND
        )

        return execution_status, self.keyword

    @service_check("grammar_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def refactor_text(self, text: str) -> str:
        request = Grammar.Request(text=text)
        future = self.grammar_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return Status.EXECUTION_SUCCESS, future.result().corrected_text

    @service_check("llm_wrapper_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def ask(self, question: str) -> str:
        request = LLMWrapper.Request(question=question)
        future = self.llm_wrapper_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return Status.EXECUTION_SUCCESS, future.result().answer

    @service_check("command_interpreter_client", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def command_interpreter(self, text: str) -> CommandInterpreter.Response:
        request = CommandInterpreter.Request(text=text)
        future = self.command_interpreter_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return Status.EXECUTION_SUCCESS, future.result().commands

    @service_check("common_interest_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def common_interest(self, person1, interest1, person2, interest2, remove_thinking=True):
        request = CommonInterest.Request(
            person1=person1, interests1=interest1, person2=person2, interests2=interest2
        )
        future = self.common_interest_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        result = future.result().common_interest

        if remove_thinking:
            result = re.sub(r"<think>.*?</think>", "", result, flags=re.DOTALL)

        return Status.EXECUTION_SUCCESS, result

    @service_check("is_positive_service", (Status.SERVICE_CHECK, False), TIMEOUT)
    def is_positive(self, text):
        request = IsPositive.Request(text=text)
        future = self.is_positive_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().is_positive

    def _add_to_collection(self, document: list, metadata: str, collection: str) -> str:
        request = AddEntry.Request(document=document, metadata=metadata, collection=collection)
        future = self.add_item_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return "Success" if future.result().success else f"Failed: {future.result().message}"

    def add_item(self, document: list, metadata: str) -> str:
        return self._add_to_collection(document, metadata, "items")

    def add_location(self, document: list, metadata: str) -> str:
        return self._add_to_collection(document, metadata, "locations")

    def _query_(self, query: str, collection: str, top_k: int = 1) -> list[str]:
        # Wrap the query in a list so that the field receives a sequence of strings.
        request = QueryEntry.Request(query=[query], collection=collection, topk=top_k)
        future = self.query_item_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().results

    def query_item(self, query: str, top_k: int = 1) -> list[str]:
        return self._query_(query, "items", top_k)

    def query_location(self, query: str, top_k: int = 1) -> list[str]:
        return self._query_(query, "locations", top_k)

    def categorize_objects(
        self, table_objects: list[str], shelves: dict[int, list[str]]
    ) -> tuple[Status, dict[int, list[str]], dict[int, list[str]]]:
        """
        Categorize objects based on their shelf levels.

        Args:
            table_objects (list[str]): List of objects on the table.
            shelves (dict[int, list[str]]): Dictionary mapping shelf levels to object names.

        Returns:
            dict[int, list[str]]: Dictionary mapping shelf levels to categorized objects.
        """
        try:
            request = CategorizeShelves.Request(table_objects=table_objects, shelves=shelves)
            future = self.categorize_service.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            res: CategorizeShelves.Response = future.result()
            categorized_shelves = eval(res.categorized_shelves)
            categorized_shelves = {int(k): v for k, v in categorized_shelves.items()}
            objects_to_add = eval(res.objects_to_add)
            objects_to_add = {int(k): v for k, v in objects_to_add.items()}
        except Exception as e:
            self.node.get_logger().error(f"Error: {e}")
            return Status.EXECUTION_ERROR, {}, {}
        return Status.EXECUTION_SUCCESS, categorized_shelves, objects_to_add


if __name__ == "__main__":
    rclpy.init()
    node = Node("hri_tasks")
    vision_tasks = HRITasks(node)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
