#!/usr/bin/env python3

"""
HRI Subtask manager
"""

import json
import os
import re
from datetime import datetime
from typing import List, Union

import rclpy
from ament_index_python.packages import get_package_share_directory
from frida_constants.hri_constants import (
    ADD_ENTRY_SERVICE,
    CATEGORIZE_SERVICE,
    COMMON_INTEREST_SERVICE,
    EXTRACT_DATA_SERVICE,
    GRAMMAR_SERVICE,
    HOTWORD_SERVICE_NAME,
    IS_NEGATIVE_SERVICE,
    IS_POSITIVE_SERVICE,
    LLM_WRAPPER_SERVICE,
    QUERY_ENTRY_SERVICE,
    RAG_SERVICE,
    SPEAK_SERVICE,
    STT_SERVICE_NAME,
    USEFUL_AUDIO_NODE_NAME,
    WAKEWORD_TOPIC,
)
from frida_interfaces.srv import (
    STT,
    AddEntry,
    AnswerQuestion as AnswerQuestionLLM,
    CategorizeShelves,
    CommonInterest,
    ExtractInfo,
    Grammar,
    IsNegative,
    IsPositive,
    LLMWrapper,
    QueryEntry,
    Speak,
    UpdateHotwords,
)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import String
from utils.baml_client.sync_client import b
from utils.baml_client.types import AnswerQuestion
from utils.baml_client.types import (
    CommandListLLM,
    Count,
    FindPerson,
    FindPersonByName,
    FollowPersonUntil,
    GetPersonInfo,
    GetVisualInfo,
    GiveObject,
    GoTo,
    GuidePersonTo,
    PickObject,
    PlaceObject,
    SayWithContext,
)
from utils.decorators import service_check
from utils.logger import Logger
from utils.status import Status
from utils.task import Task

from subtask_managers.subtask_meta import SubtaskMeta

InterpreterAvailableCommands = Union[
    CommandListLLM,
    GoTo,
    PickObject,
    FindPersonByName,
    FindPerson,
    Count,
    GetPersonInfo,
    GetVisualInfo,
    AnswerQuestion,
    FollowPersonUntil,
    GuidePersonTo,
    GiveObject,
    PlaceObject,
    SayWithContext,
]

TIMEOUT = 5.0


def confirm_query(interpreted_text, target_info):
    return f"Did you say {target_info}?"


class HRITasks(metaclass=SubtaskMeta):
    """Class to manage the vision tasks"""

    def __init__(self, task_manager: Node, config=None, task=Task.RECEPTIONIST) -> None:
        self.node = task_manager
        self.keyword = ""
        self.speak_service = self.node.create_client(Speak, SPEAK_SERVICE)
        self.hear_service = self.node.create_client(STT, STT_SERVICE_NAME)
        self.extract_data_service = self.node.create_client(ExtractInfo, EXTRACT_DATA_SERVICE)
        self.task = task
        self.grammar_service = self.node.create_client(Grammar, GRAMMAR_SERVICE)
        self.common_interest_service = self.node.create_client(
            CommonInterest, COMMON_INTEREST_SERVICE
        )
        self.is_positive_service = self.node.create_client(IsPositive, IS_POSITIVE_SERVICE)
        self.is_negative_service = self.node.create_client(IsNegative, IS_NEGATIVE_SERVICE)
        self.display_publisher = self.node.create_publisher(String, "/hri/display/change_video", 10)

        self.query_item_client = self.node.create_client(QueryEntry, QUERY_ENTRY_SERVICE)
        self.add_item_client = self.node.create_client(AddEntry, ADD_ENTRY_SERVICE)
        self.llm_wrapper_service = self.node.create_client(LLMWrapper, LLM_WRAPPER_SERVICE)
        self.categorize_service = self.node.create_client(CategorizeShelves, CATEGORIZE_SERVICE)
        self.hotwords_service = self.node.create_client(UpdateHotwords, HOTWORD_SERVICE_NAME)
        self.keyword_client = self.node.create_subscription(
            String, WAKEWORD_TOPIC, self._get_keyword, 10
        )

        self.useful_audio_params = self.node.create_client(
            SetParameters, f"/{USEFUL_AUDIO_NODE_NAME}/set_parameters"
        )

        self.answer_question_service = self.node.create_client(AnswerQuestionLLM, RAG_SERVICE)

        all_services = {
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
        }

        self.services = {
            Task.RECEPTIONIST: all_services,
            Task.GPSR: all_services,
            Task.HELP_ME_CARRY: all_services,
            Task.STORING_GROCERIES: all_services,
        }

        package_share_directory = get_package_share_directory("frida_constants")
        file_path = os.path.join(package_share_directory, "data/positive.json")
        with open(file_path, "r") as file:
            self.positive = json.load(file)["affirmations"]

        self.setup_services()
        Logger.success(self.node, f"hri_tasks initialized with task {self.task}")

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
        Logger.info(self.node, f"Sending to saying service: {text}")

        request = Speak.Request(text=text)

        future = self.speak_service.call_async(request)

        if wait:
            rclpy.spin_until_future_complete(self.node, future)
            return Status.EXECUTION_SUCCESS if future.result().success else Status.EXECUTION_ERROR
        Logger.info(self.node, "Saying service finished executing")

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
        Logger.info(
            self.node, f"Sending to extract data service: query={query}, text={complete_text}"
        )

        request = ExtractInfo.Request(data=query, full_text=complete_text, context=context)
        future = self.extract_data_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        execution_status = (
            Status.EXECUTION_SUCCESS if len(future.result().result) > 0 else Status.TARGET_NOT_FOUND
        )

        if execution_status == Status.EXECUTION_SUCCESS:
            Logger.info(
                self.node,
                f"extract_data result: {future.result().result}",
            )
        else:
            Logger.warn(self.node, "extract_data: no data found")

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
    def hear(self, min_audio_length=1.0, max_audio_length=10.0) -> str:
        if min_audio_length > 0:
            self.set_double_param("MIN_AUDIO_DURATION", float(min_audio_length))

        if max_audio_length > 0:
            self.set_double_param("MAX_AUDIO_DURATION", float(max_audio_length))

        request = STT.Request()

        future = self.hear_service.call_async(request)
        Logger.info(
            self.node,
            "Hearing from the user...",
        )
        rclpy.spin_until_future_complete(self.node, future)

        execution_status = (
            Status.EXECUTION_SUCCESS
            if len(future.result().text_heard) > 0
            else Status.TARGET_NOT_FOUND
        )

        if execution_status == Status.EXECUTION_SUCCESS:
            Logger.info(
                self.node,
                f"hearing result: {future.result().text_heard}",
            )
        else:
            Logger.warn(self.node, "hearing: no text heard")

        return execution_status, future.result().text_heard

    @service_check("hotwords_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def set_hotwords(self, hotwords) -> str:
        Logger.info(
            self.node,
            "Setting hotwords: " + str(hotwords),
        )
        request = UpdateHotwords.Request(hotwords=hotwords)
        future = self.hotwords_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        execution_status = (
            Status.EXECUTION_SUCCESS if future.result().success else Status.EXECUTION_ERROR
        )

        return execution_status, ""

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
        Logger.info(
            self.node,
            "Asking for confirmation: " + question,
        )
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
                        # check if positive word is in the interpreted text, if not, check if the text is positive with llm
                        for word in self.positive:
                            if word in interpret_text.lower():
                                return Status.EXECUTION_SUCCESS, "yes"

                        if self.is_positive(interpret_text)[1]:
                            return Status.EXECUTION_SUCCESS, "yes"
                        return Status.EXECUTION_SUCCESS, "no"
        Logger.info(
            self.node,
            "Confirmation timed out for: " + question,
        )
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
        skip_extract_data: bool = False,
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

            if s == Status.EXECUTION_SUCCESS:
                if not skip_extract_data:
                    s, target_info = self.extract_data(query, interpreted_text, context)
                else:
                    s = Status.EXECUTION_SUCCESS
                    target_info = interpreted_text

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

        Logger.warn(
            self.node,
            "Ask and confirm timed out for question: " + question,
        )
        return Status.TIMEOUT, ""

    def interpret_keyword(self, keywords: list[str], timeout: float) -> str:
        start_time = self.node.get_clock().now()
        self.keyword = ""
        Logger.info(
            self.node,
            f"Listening for keywords: {str(keywords)}",
        )
        while (
            self.keyword not in keywords
            and ((self.node.get_clock().now() - start_time).nanoseconds / 1e9) < timeout
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        execution_status = (
            Status.EXECUTION_SUCCESS if self.keyword in keywords else Status.TARGET_NOT_FOUND
        )

        if execution_status == Status.EXECUTION_SUCCESS:
            Logger.info(
                self.node,
                f"Keyword recognized: {self.keyword}",
            )
        else:
            Logger.warn(
                self.node,
                "interpret_keyword: no keyword recognized",
            )

        return execution_status, self.keyword

    @service_check("grammar_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def refactor_text(self, text: str) -> str:
        request = Grammar.Request(text=text)
        future = self.grammar_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return Status.EXECUTION_SUCCESS, future.result().corrected_text

    def command_interpreter(self, text: str) -> List[InterpreterAvailableCommands]:
        Logger.info(
            self.node,
            "Received command for interpretation: " + text,
        )
        command_list = b.GenerateCommandList(request=text)

        Logger.info(
            self.node,
            "command_interpreter result: " + str(command_list.commands),
        )

        return Status.EXECUTION_SUCCESS, command_list.commands

    @service_check("useful_audio_params", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def set_double_param(self, name, value):
        param = Parameter()

        param.name = name

        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE)

        param.value.double_value = value

        request = SetParameters.Request()

        request.parameters = [param]

        future = self.useful_audio_params.call_async(request)

        while not future.done():
            # self.node.get_logger().info(f"Setting parameter {name} to {value}")
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if future.result() is not None:
            pass
            # self.node.get_logger().info(f"Parameter {name} set to {value}")
        else:
            self.node.get_logger().error(f"Failed to set parameter {name}")

    @service_check("common_interest_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def common_interest(self, person1, interest1, person2, interest2, remove_thinking=True):
        Logger.info(
            self.node,
            f"Finding common interest between {person1}({interest1}) and {person2}({interest2})",
        )
        request = CommonInterest.Request(
            person1=person1, interests1=interest1, person2=person2, interests2=interest2
        )
        future = self.common_interest_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        result = future.result().common_interest

        if remove_thinking:
            result = re.sub(r"<think>.*?</think>", "", result, flags=re.DOTALL)

        Logger.info(
            self.node, f"Common interest computed between {person1} and {person2}: {result}"
        )

        return Status.EXECUTION_SUCCESS, result

    @service_check("is_positive_service", (Status.SERVICE_CHECK, False), TIMEOUT)
    def is_positive(self, text, async_call=False):
        Logger.info(self.node, f"Checking if text is positive: {text}")
        request = IsPositive.Request(text=text)
        future = self.is_positive_service.call_async(request)
        if async_call:
            return future
        rclpy.spin_until_future_complete(self.node, future)
        Logger.info(self.node, f"is_positive result ({text}): {future.result().is_positive}")
        return Status.EXECUTION_SUCCESS, future.result().is_positive

    @service_check("is_negative_service", (Status.SERVICE_CHECK, False), TIMEOUT)
    def is_negative(self, text, async_call=False):
        Logger.info(self.node, f"Checking if text is negative: {text}")
        request = IsNegative.Request(text=text)
        future = self.is_negative_service.call_async(request)
        if async_call:
            return future
        rclpy.spin_until_future_complete(self.node, future)
        Logger.info(self.node, f"is_negative result ({text}): {future.result().is_negative}")
        return Status.EXECUTION_SUCCESS, future.result().is_negative

    @service_check("answer_question_service", (Status.SERVICE_CHECK, "", 0.5), TIMEOUT)
    def answer_question(
        self, question: str, top_k: int = None, threshold: float = None, collections: list = None
    ) -> tuple[Status, str]:
        """
        Method to answer a question using the RAG service.

        Args:
            question: The question to answer
            top_k: Number of top results to consider (default: use service default)
            threshold: Similarity threshold for including context (default: use service default)
            collections: List of collections to search in (default: use service default)

        Returns:
            Status: The status of the execution
            str: The answer to the question
            float: Confidence score of the answer
        """
        Logger.info(self.node, f"Sending question to RAG service: {question}")

        request = AnswerQuestionLLM.Request(
            question=question,
            topk=top_k if top_k is not None else 0,
            threshold=threshold if threshold is not None else 0.0,
            collections=collections if collections is not None else [],
        )

        future = self.answer_question_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        result = future.result()
        if result.success:
            Logger.info(self.node, f"RAG answer received with score: {result.score}")
            return Status.EXECUTION_SUCCESS, result.response, result.score
        else:
            Logger.warn(self.node, f"RAG service failed: {result.response}")
            return Status.EXECUTION_ERROR, result.response, result.score

    # /////////////////embeddings services/////
    def add_command_history(self, command: InterpreterAvailableCommands, result, status):
        collection = "command_history"

        document = [command.action]
        metadata = [
            {
                "action": command.action,
                "command": str(command),
                "result": result,
                "status": status,
                "timestamp": datetime.now().isoformat(),
            }
        ]

        request = AddEntry.Request(
            document=document, metadata=json.dumps(metadata), collection=collection
        )
        future = self.add_item_client.call_async(request)

        def callback(fut):
            try:
                response = fut.result()
                self.node.get_logger().info(f"Command history saved: {response}")
            except Exception as e:
                self.node.get_logger().error(f"Failed to save command history: {e}")

        future.add_done_callback(callback)
        return Status.EXECUTION_SUCCESS

    def add_item(self, document: list, metadata: str) -> list[str]:
        return self._add_to_collection(document, metadata, "items")

    def add_location(self, document: list, metadata: str) -> list[str]:
        return self._add_to_collection(document, metadata, "locations")

    def query_item(self, query: str, top_k: int = 1) -> list[str]:
        return self._query_(query, "items", top_k)

    def query_location(self, query: str, top_k: int = 1) -> list[str]:
        return self._query_(query, "locations", top_k)

    def find_closest(self, documents: list, query: str, top_k: int = 1) -> list[str]:
        """
        Method to find the closest item to the query.
        Args:
            documents: the documents to search among
            query: the query to search for
        Returns:
            Status: the status of the execution
            list[str]: the results of the query
        """
        Logger.info(self.node, f"Finding closest items to: {query} in {str(documents)}")
        self._add_to_collection(document=documents, metadata="", collection="closest_items")
        self.node.get_logger().info(f"Adding closest items: {documents}")
        Results = self._query_(query, "closest_items", top_k)
        Results = self.get_name(Results)
        Logger.info(self.node, f"find_closest result({query}): {str(Results)}")
        return Status.EXECUTION_SUCCESS, Results

    @service_check("llm_wrapper_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def answer_with_context(self, question: str, context: str) -> str:
        """
        Method to answer a question with context.
        Args:
            question: the question to answer
            context: the context to use
        Returns:
            Status: the status of the execution
            str: the answer to the question
        """
        request = LLMWrapper.Request(question=question, context=context)
        future = self.llm_wrapper_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return Status.EXECUTION_SUCCESS, future.result().answer

    def query_command_history(self, query: str, top_k: int = 1):
        """
        Method to query the command history collection.
        Args:
            query: the query to search for
        Returns:
            Status: the status of the execution
            list[str]: the results of the query
        """
        return self._query_(query, "command_history", top_k)

    # /////////////////helpers/////
    def _query_(self, query: str, collection: str, top_k: int = 1) -> tuple[Status, list[str]]:
        # Wrap the query in a list so that the field receives a sequence of strings.
        request = QueryEntry.Request(query=[query], collection=collection, topk=top_k)
        future = self.query_item_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if collection == "command_history":
            results_loaded = json.loads(future.result().results[0])
            sorted_results = sorted(
                results_loaded["results"], key=lambda x: x["metadata"]["timestamp"], reverse=True
            )
            results_list = sorted_results[:top_k]
        else:
            results = future.result().results
            results_loaded = json.loads(results[0])
            results_list = results_loaded["results"]
        return Status.EXECUTION_SUCCESS, results_list

    def _add_to_collection(self, document: list, metadata: str, collection: str) -> str:
        request = AddEntry.Request(document=document, metadata=metadata, collection=collection)
        future = self.add_item_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return (
            Status.EXECUTION_SUCCESS,
            "Success" if future.result().success else f"Failed: {future.result().message}",
        )

    def get_context(self, query_result):
        return self.get_metadata_key(query_result, "context")

    # TODO: Fix since we removed the complement from the command
    def get_complement(self, query_result):
        return self.get_metadata_key(query_result, "complement")

    def get_command(self, query_result):
        return self.get_metadata_key(query_result, "command")

    def get_characteristic(self, query_result):
        return self.get_metadata_key(query_result, "characteristic")

    def get_result(self, query_result):
        return self.get_metadata_key(query_result, "result")

    def get_status(self, query_result):
        return self.get_metadata_key(query_result, "status")

    def get_name(self, query_result):
        return self.get_metadata_key(query_result, "original_name")

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
        Logger.info(self.node, "Sending request to categorize_objects")

        try:
            request = CategorizeShelves.Request()
            table_objects = [String(data=str(obj)) for obj in table_objects]
            request = CategorizeShelves.Request(
                table_objects=table_objects, shelves=String(data=str(shelves))
            )

            future = self.categorize_service.call_async(request)
            Logger.info(self.node, "generated request")
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=20)
            res = future.result()
            Logger.info(self.node, "request finished")
            # if res.status != Status.EXECUTION_SUCCESS:
            #     Logger.error(self.node, f"Error in categorize_objects: {res.status}")
            #     return Status.EXECUTION_ERROR, {}, {}

            categorized_shelves = eval(res.categorized_shelves.data)
            categorized_shelves = {int(k): v for k, v in categorized_shelves.items()}
            objects_to_add = eval(res.objects_to_add.data)
            objects_to_add = {int(k): v for k, v in objects_to_add.items()}
        except Exception as e:
            self.node.get_logger().error(f"Error: {e}")
            return Status.EXECUTION_ERROR, {}, {}

        Logger.info(self.node, "Finished executing categorize_objects")

        return Status.EXECUTION_SUCCESS, categorized_shelves, objects_to_add

    def get_subarea(self, query_result):
        return self.get_metadata_key(query_result, "subarea")

    def get_area(self, query_result):
        return self.get_metadata_key(query_result, "area")

    def get_metadata_key(self, query_result, field: str):
        """
        Extracts the field from the metadata of a query result.

        Args:
            query_result (tuple): The query result tuple (status, list of JSON strings)

        Returns:
            list: The 'context' field from metadata, or empty string if not found
        """
        try:
            key_list = []
            query_result = query_result[1]
            for result in query_result:
                metadata = result["metadata"]
                if isinstance(metadata, list) and metadata:
                    metadata = metadata[0]
                result_key = metadata.get(field, "")  # safely get 'field'
                key_list.append(result_key)
            return key_list
        except (IndexError, KeyError, json.JSONDecodeError) as e:
            self.node.get_logger().error(f"Failed to extract context: {str(e)}")
            return ""

    def publish_display_topic(self, topic: str):
        self.display_publisher.publish(String(data=topic))
        Logger.info(self.node, f"Published display topic: {topic}")

    def get_timestamps(self, query_result):
        return self.get_metadata_key(query_result, "timestamp")


if __name__ == "__main__":
    rclpy.init()
    node = Node("hri_tasks")
    vision_tasks = HRITasks(node)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
