#!/usr/bin/env python3

"""
HRI Subtask manager
"""

import json
import os
import re
from enum import Enum

# from datetime import datetime
from typing import List, Union

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from embeddings.postgres_adapter import PostgresAdapter
from frida_constants.hri_constants import (  # ADD_ENTRY_SERVICE,
    CATEGORIZE_SERVICE,
    COMMAND_INTERPRETER_SERVICE,
    COMMON_INTEREST_SERVICE,
    EXTRACT_DATA_SERVICE,
    GRAMMAR_SERVICE,
    IS_NEGATIVE_SERVICE,
    IS_POSITIVE_SERVICE,
    LLM_WRAPPER_SERVICE,
    QUERY_ENTRY_SERVICE,
    RAG_SERVICE,
    RESPEAKER_LIGHT_TOPIC,
    SPEAK_SERVICE,
    STT_ACTION_SERVER_NAME,
    WAKEWORD_TOPIC,
)
from frida_interfaces.action import SpeechStream
from frida_interfaces.srv import AnswerQuestion as AnswerQuestionLLM
from frida_interfaces.srv import (  # AddEntry,
    CategorizeShelves,
    CommandInterpreter,
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


def contains_any(text: List[str], keywords: List[str]) -> bool:
    """
    Check if any of the keywords are present in the text.

    Args:
        text (List[str]): The text to check.
        keywords (List[str]): The list of keywords to search for.

    Returns:
        bool: True if any keyword is found in the text, False otherwise.
    """

    for keyword in keywords:
        for word in text:
            if keyword.lower() == word.lower():
                return keyword
    return None


def remove_punctuation(text: str) -> str:
    """
    Remove punctuation from the text.

    Args:
        text (str): The text to process.

    Returns:
        str: The text without punctuation.
    """
    return re.sub(r"[^\w\s]", "", text).strip().lower()


def format_transcription(text: str) -> str:
    """Format the interpreted text to remove punctuation and convert to lowercase."""
    return remove_punctuation(text).split(" ")


class AudioStates(Enum):
    SAYING = "saying"
    LISTEN = "listening"
    IDLE = "idle"

    @classmethod
    def respeaker_light(cls, state):
        if state == AudioStates.SAYING:
            return "speak"
        elif state == AudioStates.LISTEN:
            return "listen"
        elif state == AudioStates.IDLE:
            return "off"
        else:
            raise ValueError(f"Unknown audio state: {state}")


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
        # self.add_item_client = self.node.create_client(AddEntry, ADD_ENTRY_SERVICE)
        self.pg = PostgresAdapter()
        self.llm_wrapper_service = self.node.create_client(LLMWrapper, LLM_WRAPPER_SERVICE)
        self.categorize_service = self.node.create_client(CategorizeShelves, CATEGORIZE_SERVICE)
        self.keyword_client = self.node.create_subscription(
            String, WAKEWORD_TOPIC, self._get_keyword, 10
        )

        self.current_transcription = ""
        self.last_hotwords = ""
        self._active_goals = []
        self.hear_multi_service = self.node.create_client(
            HearMultiThread, "/integration/multi_stop"
        )

        self.answer_question_service = self.node.create_client(AnswerQuestionLLM, RAG_SERVICE)

        self.command_interpreter_service = self.node.create_client(
            CommandInterpreter, COMMAND_INTERPRETER_SERVICE
        )
        self.audio_state_publisher = self.node.create_publisher(String, "AudioState", 10)
        self.respeaker_light_publisher = self.node.create_publisher(
            String, RESPEAKER_LIGHT_TOPIC, 10
        )

        self._action_client = ActionClient(self.node, SpeechStream, STT_ACTION_SERVER_NAME)

        all_services = {
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

        gpsr_services = {
            "command_interpreter_service": {
                "client": self.command_interpreter_service,
                "type": "service",
            },
        }

        self.services = {
            Task.RECEPTIONIST: all_services,
            Task.GPSR: all_services | gpsr_services,
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

    @service_check("_action_client", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def hear(
        self, hotwords="", silence_time=2.0, start_silence_time=4.0, max_audio_length=13.0
    ) -> str:
        Logger.info(
            self.node,
            "Hearing from the user...",
        )

        accepted_future = self.hear_streaming(
            hotwords=hotwords,
            silence_time=silence_time,
            start_silence_time=start_silence_time,
            timeout=max_audio_length,
        )

        rclpy.spin_until_future_complete(
            self.node, accepted_future, timeout_sec=max_audio_length + 1
        )

        goal_future = accepted_future.result().get_result_async()

        # Add an extra second to ensure the action server has enough time to process the request
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=max_audio_length + 1)
        self.cancel_hear_action()

        execution_status = (
            Status.EXECUTION_SUCCESS
            if len(goal_future.result().result.transcription) > 0
            else Status.TARGET_NOT_FOUND
        )

        if execution_status == Status.EXECUTION_SUCCESS:
            Logger.info(
                self.node,
                f"hearing result: {goal_future.result().result.transcription}",
            )
        else:
            Logger.warn(self.node, "hearing result: no text heard")

        return execution_status, goal_future.result().result.transcription

    def hear_streaming(
        self,
        timeout: float = 13.0,
        hotwords: str = "",
        silence_time: float = 2.0,
        start_silence_time: float = 4.0,
    ):
        """Method to hear streaming audio from the user.

        Note: the caller must also call `self.cancel_hear_action()` or `self.set_light_state(AudioStates.IDLE)` to update respeaker light state.

        Args:
            timeout (float): The maximum time to stop the transcription.
            hotwords (str): Hotwords to improve the transcription accuracy.
            silence_time (float): The time to wait after the last interpreted word to stop the transcription. i.e. if no words are heard for this time, the transcription will stop.
            start_silence_time (float): The minimum duration of the transcription before hearing any words. Useful to handle initial silence in audio.
        """
        # Cancel other actions if they are running
        self.cancel_hear_action()
        self.set_light_state(AudioStates.LISTEN)

        self.current_transcription = ""

        goal_msg = SpeechStream.Goal()
        goal_msg.timeout = float(timeout)
        goal_msg.hotwords = hotwords
        self.last_hotwords = hotwords
        goal_msg.silence_time = float(silence_time)
        goal_msg.start_silence_time = float(start_silence_time)

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        future.add_done_callback(lambda f: self._active_goals.append(f.result()))

        return future

    def feedback_callback(self, feedback_msg):
        self.current_transcription = feedback_msg.feedback.current_transcription
        self.node.get_logger().info("Received feedback: {0}".format(self.current_transcription))

    def set_light_state(self, state: AudioStates):
        """
        Method to set the light state of the respeaker.
        Args:
            state: The state of the light.
        """
        self.respeaker_light_publisher.publish(String(data=AudioStates.respeaker_light(state)))
        self.audio_state_publisher.publish(String(data=state.value))

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
                accepted_future = self.hear_streaming(timeout=wait_between_retries)

                rclpy.spin_until_future_complete(
                    self.node, accepted_future, timeout_sec=wait_between_retries + 1
                )
                goal_future = accepted_future.result().get_result_async()

                while not goal_future.done():
                    if contains_any(
                        format_transcription(self.current_transcription), self.positive
                    ):
                        self.cancel_hear_action()
                        return Status.EXECUTION_SUCCESS, "yes"
                    if "no" in format_transcription(self.current_transcription):
                        self.cancel_hear_action()
                        return Status.EXECUTION_SUCCESS, "no"
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                # Add an extra second to ensure the action server has enough time to process the request
                rclpy.spin_until_future_complete(
                    self.node, goal_future, timeout_sec=wait_between_retries + 1
                )
                self.cancel_hear_action()

                # If the transcription is equal to the last hotwords, consider that no text was heard: when the audio is too short or only silence, the transcription can be equal to the hotwords.
                if (
                    len(goal_future.result().result.transcription) > 0
                    and goal_future.result().result.transcription != self.last_hotwords
                ):
                    if (
                        contains_any(
                            format_transcription(self.current_transcription), self.positive
                        )
                        or self.is_positive(self.current_transcription)[1]
                    ):
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
        skip_confirmation: bool = False,
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
            hear_status, interpreted_text = self.hear()

            if hear_status == Status.EXECUTION_SUCCESS:
                if not skip_extract_data:
                    s, target_info = self.extract_data(query, interpreted_text, context)
                else:
                    s = Status.TARGET_NOT_FOUND

                if s == Status.TARGET_NOT_FOUND:
                    target_info = interpreted_text

                if skip_confirmation:
                    return Status.EXECUTION_SUCCESS, target_info

                # Determine the confirmation question
                if callable(confirm_question):
                    confirmation_text = confirm_question(interpreted_text, target_info)
                else:
                    confirmation_text = confirm_question

                s, confirmation = self.confirm(confirmation_text, use_hotwords, 3)

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
        self.cancel_hear_action()

        self.hear_streaming(timeout=timeout, silence_time=timeout)

        start_time = self.node.get_clock().now()
        self.keyword = ""

        Logger.info(
            self.node,
            f"Listening for keywords: {str(keywords)}",
        )

        while (
            self.keyword not in keywords
            and not contains_any(format_transcription(self.current_transcription), keywords)
            and ((self.node.get_clock().now() - start_time).nanoseconds / 1e9) < timeout
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        keyword_listened = contains_any(format_transcription(self.current_transcription), keywords)

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

        self.cancel_hear_action()
        return execution_status, keyword_listened

    @service_check("grammar_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def refactor_text(self, text: str) -> str:
        request = Grammar.Request(text=text)
        future = self.grammar_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return Status.EXECUTION_SUCCESS, future.result().corrected_text

    @service_check("", (Status.SERVICE_CHECK, ("coke", "left")), TIMEOUT)
    def get_location_orientation(self) -> tuple[Status, tuple[str, str]]:
        """
        Method to get the location and orientation of where to place the object
        Returns:
            tuple[Status, tuple[str, str]]: A tuple containing the status and a tuple with the location and orientation.
            The location is a string representing the location (e.g., "coke") and the orientation is a string representing the direction (e.g., "left").
        """
        current_retries_location = 0
        selected_loc = None
        closest_by_name = None
        closest_by_description = None

        while current_retries_location < 3:
            current_retries_location += 1

            s, location = self.ask_and_confirm(
                "Where do you want me to place the object? Please describe the location.",
                "location",
                use_hotwords=False,
                retries=3,
                min_wait_between_retries=5,
                skip_extract_data=True,
                skip_confirmation=True,
            )

            closest_by_name, closest_by_description = self.pg.get_hand_items(location)

            _, confirmation = self.confirm(
                "Should I place it near " + closest_by_description[0].name + "?",
                use_hotwords=True,
                retries=3,
            )

            if confirmation == "yes":
                selected_loc = closest_by_description[0].name
                self.say(
                    f"Thanks for confirming the location. Near {selected_loc}",
                    wait=False,
                )
            else:
                selected_loc = closest_by_name[0].name

        current_retries_orientation = 0
        selected_orientation = None

        while current_retries_orientation < 3:
            current_retries_orientation += 1

            s, location = self.ask_and_confirm(
                "Where do you want me to place it with respect to the provided location (left, right, front, back, top or just nearby)?",
                "LLM_orientation",
                use_hotwords=False,
                retries=3,
                min_wait_between_retries=5,
                skip_extract_data=True,
                skip_confirmation=True,
            )

            if s != Status.EXECUTION_SUCCESS:
                self.say("I didn't understand the orientation. Let's try again.")
                Logger.warn(self.node, "Failed to get orientation, trying again")
                continue

            s, closest = self.find_closest(
                ["left", "right", "front", "back", "top", "nearby"], location
            )

            prefix = "on the " if closest != "nearby" else ""

            _, confirmation = self.confirm(
                "Should I place it " + prefix + closest + "?",
                use_hotwords=True,
                retries=3,
            )

            if confirmation == "yes":
                selected_orientation = closest
                self.say(
                    "Thanks for confirming the orientation",
                    wait=False,
                )
            else:
                selected_loc = closest_by_name[0].name

        matching_items = [item for item in closest_by_description if item.name == selected_loc]

        if len(matching_items) > 1:
            self.display_publisher.publish(String(data=json.dumps(matching_items)))
            s, location = self.ask_and_confirm(
                "I found several locations with the specified name. Please select the location by saying the color of the point.",
                "LLM_color",
                use_hotwords=False,
                retries=3,
                min_wait_between_retries=5,
                skip_extract_data=False,
            )
            s, closest = self.find_closest([item.color for item in matching_items], location)
            self.display_publisher.publish(String(data="cancel"))

            matching_items = [item for item in matching_items if item.color == closest]

        return (Status.EXECUTION_SUCCESS, matching_items[0], selected_orientation)

    def command_interpreter(
        self, text: str, is_async=False
    ) -> List[InterpreterAvailableCommands] | Future:
        """
        Interprets a command and returns a list of actions to be executed.

        Args:
            text (str): The command text to interpret.
            is_async (bool): Whether to run the interpretation asynchronously.
        Returns:
            Status: The status of the execution.
            List[InterpreterAvailableCommands]: The list of interpreted commands.

        If is_async is True, returns a Future object that can be used to get the result later.
        """
        Logger.info(
            self.node,
            "Received command for interpretation: " + text,
        )

        if is_async:
            future = Future()

            request = CommandInterpreter.Request(text=text)
            command_interpreter_future = self.command_interpreter_service.call_async(request)

            def callback(f):
                parsed = b.parse.GenerateCommandList(f.result().unparsed_response)

                future.set_result(
                    (
                        Status.EXECUTION_SUCCESS
                        if isinstance(parsed, CommandListLLM)
                        else Status.EXECUTION_ERROR,
                        parsed.commands if isinstance(parsed, CommandListLLM) else [],
                    )
                )

            command_interpreter_future.add_done_callback(callback)
            self.node.get_logger().info("Command interpreter service called asynchronously")

            return future

        # Legacy synchronous call
        command_list = b.GenerateCommandList(request=text)

        Logger.info(
            self.node,
            "command_interpreter result: " + str(command_list.commands),
        )

        return Status.EXECUTION_SUCCESS, command_list.commands

    @service_check("common_interest_service", (Status.SERVICE_CHECK, ""), TIMEOUT)
    def common_interest(
        self, person1, interest1, person2, interest2, remove_thinking=True, is_async=False
    ):
        try:
            future = Future()
            Logger.info(
                self.node,
                f"Finding common interest between {person1}({interest1}) and {person2}({interest2})",
            )
            request = CommonInterest.Request(
                person1=person1, interests1=interest1, person2=person2, interests2=interest2
            )
            common_interest_future = self.common_interest_service.call_async(request)

            def callback(f):
                result = f.result().common_interest
                if remove_thinking:
                    result = re.sub(r"<think>.*?</think>", "", result, flags=re.DOTALL)

                Logger.info(
                    self.node, f"Common interest computed between {person1} and {person2}: {result}"
                )
                future.set_result(
                    (
                        Status.EXECUTION_SUCCESS,
                        result,
                    )
                )

            common_interest_future.add_done_callback(callback)
            if not is_async:
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=15)
                return future.result()
            return future
        except Exception as e:
            Logger.error(self.node, f"Error in common interest service: {e}")
            return Status.EXECUTION_ERROR, ""

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

    def cancel_hear_action(self):
        # Cancel all goals sent by this action client

        cancel_future = []
        for goal_handle in self._active_goals:
            future = goal_handle.cancel_goal_async()
            cancel_future.append(future)

        for f in cancel_future:
            rclpy.spin_until_future_complete(self.node, f, timeout_sec=1)

        self.set_light_state(AudioStates.IDLE)

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
        # collection = "command_history"

        # document = [command.action]
        # metadata = [
        #     {
        #         "command": str(command),
        #         "result": result,
        #         "status": status,
        #         "timestamp": datetime.now().isoformat(),
        #     }
        # ]

        # request = AddEntry.Request(
        #     document=document, metadata=json.dumps(metadata), collection=collection
        # )
        # future = self.add_item_client.call_async(request)

        # def callback(fut):
        #     try:
        #         response = fut.result()
        #         self.node.get_logger().info(
        #             f"Command history saved: {response}")
        #     except Exception as e:
        #         self.node.get_logger().error(
        #             f"Failed to save command history: {e}")

        # future.add_done_callback(callback)
        self.pg.add_command(
            action=str(command.action),
            command=str(command),
            result=result,
            status=status,
            context=type(command).__name__,
        )
        return Status.EXECUTION_SUCCESS

    def add_item(self, document: list, metadata: str) -> list[str]:
        # self.pg.add_item2(

        # )
        for doc in document:
            self.pg.add_item2(
                document=doc,
                context=metadata.get("context", ""),
            )
        return [doc for doc in document]
        # return self._add_to_collection(document, metadata, "items")

    def add_location(self, document: list, metadata: str) -> list[str]:
        for doc in document:
            self.pg.add_location2(
                document=doc,
                context=metadata.get("context", ""),
            )
        return [doc for doc in document]
        # return self._add_to_collection(document, metadata, "locations")

    def query_item(self, query: str, top_k: int = 1) -> list[str]:
        # return self._query_(query, "items", top_k)
        return self.pg.query_items(query=query, top_k=top_k)

    def query_location(self, query: str, top_k: int = 1) -> list[str]:
        # return self._query_(query, "locations", top_k)
        return self.pg.query_location(query=query, top_k=top_k)

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
        # self._add_to_collection(
        #     document=documents, metadata="", collection="closest_items")
        # Results = self._query_(query, "closest_items", top_k)
        # Results = self.get_name(Results)

        docs = [(doc, self.pg.embedding_model.encode(doc)) for doc in documents]
        emb = self.pg.embedding_model.encode(query)

        def cos_sim(x, y):
            return np.dot(x, y) / (np.linalg.norm(x) * np.linalg.norm(y))

        Results = sorted(docs, key=lambda x: cos_sim(x[1], emb), reverse=True)[:top_k]
        Results = [doc[0] for doc in Results]
        return Status.EXECUTION_SUCCESS, Results

        # Logger.info(self.node, f"find_closest result({query}): {str(Results)}")
        # return Status.EXECUTION_SUCCESS, Results

    # def find_closest_raw(self, documents: str, query: str, top_k: int = 1) -> list[str]:
    #     """
    #     Method to find the closest item to the query.
    #     Args:
    #         documents: the documents to search among
    #         query: the query to search for
    #     Returns:
    #         Status: the status of the execution
    #         list[str]: the results of the query
    #     """
    #     self._add_to_collection(
    #         document=documents, metadata="", collection="closest_items")
    #     Results = self._query_(query, "closest_items", top_k)
    #     Logger.info(self.node, f"find_closest result({query}): {str(Results)}")
    #     return Results

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
        results = self.pg.query_command_history(command=query, top_k=top_k)
        return Status.EXECUTION_SUCCESS, results
        # """
        # Method to query the command history collection.
        # Args:
        #     query: the query to search for
        # Returns:
        #     Status: the status of the execution
        #     list[str]: the results of the query
        # """
        # return self._query_(query, "command_history", top_k)

    # /////////////////helpers/////
    # def _query_(self, query: str, collection: str, top_k: int = 1) -> tuple[Status, list[str]]:
    #     # Wrap the query in a list so that the field receives a sequence of strings.
    #     request = QueryEntry.Request(
    #         query=[query], collection=collection, topk=top_k)
    #     future = self.query_item_client.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, future)
    #     if collection == "command_history":
    #         self.node.get_logger().info(
    #             f"Querying command history: {future.result().results}")
    #         results_loaded = json.loads(future.result().results[0])
    #         sorted_results = sorted(
    #             results_loaded["results"], key=lambda x: x["metadata"]["timestamp"], reverse=True
    #         )
    #         results_list = sorted_results[:top_k]
    #     else:
    #         results = future.result().results

    #         results_loaded = json.loads(results[0])
    #         results_list = results_loaded["results"]
    #     return Status.EXECUTION_SUCCESS, results_list

    # def _add_to_collection(self, document: list, metadata: str, collection: str) -> str:
    #     request = AddEntry.Request(
    #         document=document, metadata=metadata, collection=collection)
    #     future = self.add_item_client.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, future)

    #     return (
    #         Status.EXECUTION_SUCCESS,
    #         "Success" if future.result(
    #         ).success else f"Failed: {future.result().message}",
    #     )

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
