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
    IS_NEGATIVE_SERVICE,
    IS_POSITIVE_SERVICE,
    LLM_WRAPPER_SERVICE,
    QUERY_ENTRY_SERVICE,
    RAG_SERVICE,
    SPEAK_SERVICE,
    STT_ACTION_SERVER_NAME,
    WAKEWORD_TOPIC,
)
from frida_interfaces.action import SpeechStream
from frida_interfaces.srv import (
    AddEntry,
    CategorizeShelves,
    CommonInterest,
    ExtractInfo,
    Grammar,
    HearMultiThread,
    IsNegative,
    IsPositive,
    LLMWrapper,
    QueryEntry,
    Speak,
)
from frida_interfaces.srv import AnswerQuestion as AnswerQuestionLLM
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from utils.baml_client.sync_client import b
from utils.baml_client.types import (
    AnswerQuestion,
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

# set_log_level("INFO")  # Set to "ERROR" in prod


def confirm_query(interpreted_text, target_info):
    return f"Did you say {target_info}?"


def contains_any(text: str, keywords: List[str]) -> bool:
    """
    Check if any of the keywords are present in the text.

    Args:
        text (List[str]): The text to check.
        keywords (List[str]): The list of keywords to search for.

    Returns:
        bool: True if any keyword is found in the text, False otherwise.
    """

    for keyword in keywords:
        if keyword.lower() in text.lower():
            return keyword
    return None


class HRITasks(metaclass=SubtaskMeta):
    """Class to manage the vision tasks"""

    def __init__(self, task_manager: Node, config=None, task=Task.RECEPTIONIST) -> None:
        self.node = task_manager
        self.keyword = ""
        self.speak_service = self.node.create_client(Speak, SPEAK_SERVICE)
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
        self.keyword_client = self.node.create_subscription(
            String, WAKEWORD_TOPIC, self._get_keyword, 10
        )

        self.current_transcription = ""
        self.hear_multi_service = self.node.create_client(
            HearMultiThread, "/integration/multi_stop"
        )

        self.answer_question_service = self.node.create_client(AnswerQuestionLLM, RAG_SERVICE)

        self._action_client = ActionClient(self.node, SpeechStream, STT_ACTION_SERVER_NAME)

        all_services = {
            # "say": {
            #     "client": self.speak_service,
            #     "type": "service",
            # },
            # "extract_data_service": {
            #     "client": self.extract_data_service,
            #     "type": "service",
            # },
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

        # return Status.EXECUTION_SUCCESS

        request = Speak.Request(text=text)

        future = self.speak_service.call_async(request)

        if wait:
            rclpy.spin_until_future_complete(self.node, future)
            return Status.EXECUTION_SUCCESS if future.result().success else Status.EXECUTION_ERROR
        Logger.info(self.node, "Saying service finished executing")

        return Status.EXECUTION_SUCCESS

    @service_check("extract_data_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def extract_data(self, query, complete_text, context="", is_async=False) -> str | Future:
        """
        Extracts data from the given query and complete text.

        Args:
            query (str): specifies what to extract from complete_text.
            complete_text (str): The complete text from which data is to be extracted.
            is_async (bool): If True, the method will return a Future object instead of waiting for the result.

        Returns:
            str: The extracted data as a string. If no data is found, an empty string is returned.
        """
        Logger.info(
            self.node, f"Sending to extract data service: query={query}, text={complete_text}"
        )

        if is_async:
            future = Future()

            request = ExtractInfo.Request(data=query, full_text=complete_text, context=context)
            extract_data_future = self.extract_data_service.call_async(request)

            def callback(f):
                future.set_result(
                    (
                        Status.EXECUTION_SUCCESS
                        if len(f.result().result) > 0
                        else Status.TARGET_NOT_FOUND,
                        f.result().result,
                    )
                )

            extract_data_future.add_done_callback(callback)
            return future

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

    def hear_multi(self, status: int) -> bool:
        request = HearMultiThread.Request()
        if status == 0:
            request.stop_service = True
            request.start_service = False
        elif status == 1:
            request.stop_service = False
            request.start_service = True
        else:
            request.stop_service = False
            request.start_service = False

        future = self.hear_multi_service.call_async(request)
        Logger.info(
            self.node,
            "Checking if stopped",
        )

        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            Logger.error(self.node, "Failed receiving status word")
            return False

        return future.result().stopped

    def _get_keyword(self, msg: String) -> None:
        try:
            data = eval(msg.data)
            self.keyword = data["keyword"]
        except Exception as e:
            self.node.get_logger().error(f"Error: {e}")
            self.keyword = ""

    @service_check("hear_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def hear(self, hotwords, silence_time=1.0, max_audio_length=10.0) -> str:
        Logger.info(
            self.node,
            "Hearing from the user...",
        )

        future = self.hear_streaming(
            hotwords=hotwords, silence_time=silence_time, timeout=max_audio_length
        )

        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)

        execution_status = (
            Status.EXECUTION_SUCCESS
            if len(future.result().transcription) > 0
            else Status.TARGET_NOT_FOUND
        )

        if execution_status == Status.EXECUTION_SUCCESS:
            Logger.info(
                self.node,
                f"hearing result: {future.result().transcription}",
            )
        else:
            Logger.warn(self.node, "hearing result: no text heard")

        return execution_status, future.result().transcription

    def hear_streaming(self, timeout: float = 10.0, hotwords: str = "", silence_time: float = 5.0):
        """Method to hear streaming audio from the user.

        Args:
            timeout (float): The maximum time to stop the transcription.
            hotwords (str): Hotwords to improve the transcription accuracy.
            silence_time (float): The time to wait after the last interpreted word to stop the transcription. i.e. if no words are heard for this time, the transcription will stop.
        """
        Logger.info(self.node, "Hearing streaming from the user...")
        self.current_transcription = ""

        goal_msg = SpeechStream.Goal()
        goal_msg.timeout = timeout
        goal_msg.hotwords = hotwords
        goal_msg.silence_time = silence_time

        return self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        self.current_transcription = feedback_msg.feedback
        self.node.get_logger().info("Received feedback: {0}".format(self.current_transcription))

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
        self.cancel_action()

        self.hear_streaming(timeout=timeout, silence_time=timeout)

        start_time = self.node.get_clock().now()
        self.keyword = ""

        Logger.info(
            self.node,
            f"Listening for keywords: {str(keywords)}",
        )

        while (
            self.keyword not in keywords
            and contains_any(self.current_transcription, self.keywords)
            and ((self.node.get_clock().now() - start_time).nanoseconds / 1e9) < timeout
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        keyword_listened = contains_any(self.current_transcription, self.keywords)

        if not keyword_listened:
            keyword_listened = self.keyword

        execution_status = (
            Status.EXECUTION_SUCCESS if keyword_listened in keywords else Status.TARGET_NOT_FOUND
        )

        if execution_status == Status.EXECUTION_SUCCESS:
            Logger.info(
                self.node,
                f"Keyword recognized: {keyword_listened}",
            )
        else:
            Logger.warn(
                self.node,
                "interpret_keyword: no keyword recognized",
            )

        self.cancel_action()
        return execution_status, keyword_listened

    @service_check("grammar_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def refactor_text(self, text: str) -> str:
        request = Grammar.Request(text=text)
        future = self.grammar_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return Status.EXECUTION_SUCCESS, future.result().corrected_text

    # TODO: Make async
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

    # TODO: Make async
    @service_check("common_interest_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def common_interest(self, person1, interest1, person2, interest2, remove_thinking=True):
        try:
            Logger.info(
                self.node,
                f"Finding common interest between {person1}({interest1}) and {person2}({interest2})",
            )
            request = CommonInterest.Request(
                person1=person1, interests1=interest1, person2=person2, interests2=interest2
            )
            future = self.common_interest_service.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=15)

            result = future.result().common_interest

            if remove_thinking:
                result = re.sub(r"<think>.*?</think>", "", result, flags=re.DOTALL)

            Logger.info(
                self.node, f"Common interest computed between {person1} and {person2}: {result}"
            )
        except Exception as e:
            Logger.error(self.node, f"Error in common interest service: {e}")
            return Status.EXECUTION_ERROR, ""

        return Status.EXECUTION_SUCCESS, result

    # TODO: Make async
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

    # TODO: Make async
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

    def cancel_action(self):
        # Cancel all goals sent by this action client
        future = self._action_client.cancel_all_goals()

        # Optionally, wait for the cancel response
        rclpy.spin_until_future_complete(self.node, future)

        # Check the result
        if future.result() is not None:
            cancel_response = future.result()
            if cancel_response.goals_canceling:
                self.node.get_logger().info(
                    f"Cancelled {len(cancel_response.goals_canceling)} goal(s)."
                )
            else:
                self.node.get_logger().info("No goals were actively being canceled.")
        else:
            self.node.get_logger().warn("Failed to cancel goals.")

    @service_check("answer_question_service", (Status.SERVICE_CHECK, "", 0.5), TIMEOUT)
    def answer_question(
        self,
        question: str,
        top_k: int = 3,
        threshold: float = 0.1,
        collections: list = ["frida_knowledge", "roborregos_knowledge", "tec_knowledge"],
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
        self._add_to_collection(document=documents, metadata="", collection="closest_items")
        Results = self._query_(query, "closest_items", top_k)
        Results = self.get_name(Results)
        Logger.info(self.node, f"find_closest result({query}): {str(Results)}")
        return Status.EXECUTION_SUCCESS, Results

    def find_closest_raw(self, documents: str, query: str, top_k: int = 1) -> list[str]:
        """
        Method to find the closest item to the query.
        Args:
            documents: the documents to search among
            query: the query to search for
        Returns:
            Status: the status of the execution
            list[str]: the results of the query
        """
        self._add_to_collection(document=documents, metadata="", collection="closest_items")
        Results = self._query_(query, "closest_items", top_k)
        Logger.info(self.node, f"find_closest result({query}): {str(Results)}")
        return Results

    # TODO: Make async
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
        self.node.get_logger().info(f"answer_with_context called with: {question}, {context}")

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
            self.node.get_logger().info(f"Querying command history: {future.result().results}")
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

    def get_command(self, query_result):
        return self.get_metadata_key(query_result, "command")

    def get_result(self, query_result):
        return self.get_metadata_key(query_result, "result")

    def get_status(self, query_result):
        return self.get_metadata_key(query_result, "status")

    def get_name(self, query_result):
        return self.get_metadata_key(query_result, "original_name")

    # TODO: Make async
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
            categories = self.get_shelves_categories(shelves)[1]
            results = self.categorize_objects_with_embeddings(categories, table_objects)

            objects_to_add = {key: value["objects_to_add"] for key, value in results.items()}
            Logger.error(self.node, f"categories {categories}")

            if "empty" in categories.values():
                # add objects to add in shelves
                for k, v in objects_to_add.items():
                    for i in v:
                        shelves[k].append(i)
                categories = self.get_shelves_categories(shelves)[1]

            Logger.error(self.node, f"THIS IS THE CATEGORIZED SHELVES: {categories}")
            categorized_shelves = {
                key: value["classification_tag"] for key, value in results.items()
            }
            for k, v in categorized_shelves.items():
                if v == "empty":
                    categorized_shelves[k] = categories[k]
        #             categorized_shelves = {
        #                 key: value["classification_tag"] for key, value in results.items()
        #             }

        except Exception as e:
            self.node.get_logger().error(f"Error: {e}")
            return Status.EXECUTION_ERROR, {}, {}

        Logger.info(self.node, "Finished executing categorize_objects")

        return Status.EXECUTION_SUCCESS, categorized_shelves, objects_to_add

    def get_shelves_categories(
        self, shelves: dict[int, list[str]]
    ) -> tuple[Status, dict[int, str]]:
        """
        Categorize objects based on their shelf levels.

        Args:
            shelves (dict[int, list[str]]): Dictionary mapping shelf levels to object names.

        Returns:
            dict[int, str]: Dictionary mapping shelf levels to its category.
        """
        Logger.info(self.node, "Sending request to categorize_objects")

        try:
            request = CategorizeShelves.Request(shelves=String(data=str(shelves)), table_objects=[])

            future = self.categorize_service.call_async(request)
            Logger.info(self.node, "generated request")
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=25)
            res = future.result()
            Logger.info(self.node, "request finished")
            Logger.info(self.node, "categorize_objects result: " + str(res))
            # if res.status != Status.EXECUTION_SUCCESS:
            #     Logger.error(self.node, f"Error in categorize_objects: {res.status}")
            #     return Status.EXECUTION_ERROR, {}, {}

            categorized_shelves = res.categorized_shelves
            categorized_shelves = {k: v for k, v in enumerate(categorized_shelves)}
        except Exception as e:
            self.node.get_logger().error(f"Error: {e}")
            return Status.EXECUTION_ERROR, {}

        Logger.info(self.node, "get_shelves_categories:" + str(categorized_shelves))

        return Status.EXECUTION_SUCCESS, categorized_shelves

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

    def categorize_object(self, categories: dict, obj: str):
        """Method to categorize a list of objects in an array of objects depending on similarity"""

        try:
            category_list = []
            categories_aux = categories.copy()
            self.node.get_logger().info(f"OBJECT TO CATEGORIZE: {obj}")
            for key in list(categories_aux.keys()):
                if categories_aux[key] == "empty":
                    self.node.get_logger().info("THERE IS AN EMPTY SHELVE")
                    del categories_aux[key]

            for category in categories_aux.values():
                category_list.append(category)

            results = self.find_closest_raw(category_list, obj)
            results_distances = results[1][0]["distance"]

            result_category = results[1][0]["document"]
            self.node.get_logger().info(f"CATEGORY PREDICTED BEFORE THRESHOLD: {result_category}")
            if "empty" in categories.values() and results_distances[0] > 1:
                result_category = "empty"

            self.node.get_logger().info(f"CATEGORY PREDICTED: {result_category}")

            key_resulted = 2
            for key in list(categories.keys()):
                if str(categories[key]) == str(result_category):
                    key_resulted = key
                else:
                    self.node.get_logger().info(
                        "THE CATEGORY PREDICTED IS NOT CONTAINED IN THE REQUEST, RETURNING SHELVE 2"
                    )

            return key_resulted

        except Exception as e:
            self.node.get_logger().error(f"FAILED TO CATEGORIZE: {obj} with error: {e}")

    def categorize_objects_with_embeddings(self, categories: dict, obj_list: list):
        """
        Categorize objects based on their embeddings."""
        self.node.get_logger().info(f"THIS IS THE CATEGORIES dict RECEIVED: {categories}")
        self.node.get_logger().info(f"THIS IS THE obj_list LIST RECEIVED: {obj_list}")
        results = {
            key: {"classification_tag": categories[key], "objects_to_add": []}
            for key in categories.keys()
        }
        for obj in obj_list:
            index = self.categorize_object(categories, obj)
            results[index]["objects_to_add"].append(obj)

        self.node.get_logger().info(f"THIS IS THE RESULTS OF THE CATEGORIZATION: {results}")
        return results


if __name__ == "__main__":
    rclpy.init()
    node = Node("hri_tasks")
    vision_tasks = HRITasks(node)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
