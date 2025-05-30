#!/usr/bin/env python3

"""Miscellanous functions that interact with an LLM."""

import json
import os
from datetime import datetime
from typing import Optional

import pytz
import rclpy
from nlp.assets.dialogs import (
    format_response,
    get_categorize_shelves_args,
    get_common_interests_dialog,
    get_is_answer_negative_args,
    get_is_answer_positive_args,
    get_previous_command_answer,
)
from openai import OpenAI
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from frida_constants.hri_constants import MODEL
from frida_interfaces.srv import (
    CategorizeShelves,
    CommonInterest,
    Grammar,
    IsNegative,
    IsPositive,
    LLMWrapper,
)

SPEECH_COMMAND_TOPIC = "/speech/raw_command"
OUT_COMMAND_TOPIC = "/stop_following"

exit_keys = ["exit", "quit", "stop", "end", "terminate", "finish", "cancel"]


CURRENT_CONTEXT = """
Today is {CURRENT_DATE}.
Your name is FRIDA (Friendly robotic interactive domestic assistant), a domestic assistant developed by RoBorregos.
RoBorregos is the representative Robotic team from Tec de Monterrey, Campus Monterrey. It has around 40 members.
You compete in the Robocup@home competition. Last summer you competed in the Netherlands, at the international competition. Last March you competed in TMR, obtaining 2nd place in Mexico.
"""


def get_context():
    timezone = pytz.timezone("America/Mexico_City")
    current_date = datetime.now(timezone).strftime("%Y-%m-%d %H:%M:%S")
    return CURRENT_CONTEXT.format(CURRENT_DATE=current_date)


class LLMUtils(Node):
    base_url: Optional[str]

    def __init__(self) -> None:
        global SPEECH_COMMAND_TOPIC, OUT_COMMAND_TOPIC
        super().__init__("llm_utils")
        self.logger = self.get_logger()
        self.logger.info("Initializing llm_utils node")

        self.declare_parameter("base_url", "None")
        self.declare_parameter("SPEECH_COMMAND_TOPIC_NAME", SPEECH_COMMAND_TOPIC)
        self.declare_parameter("OUT_COMMAND_TOPIC_NAME", OUT_COMMAND_TOPIC)
        self.declare_parameter("GRAMMAR_SERVICE", "/nlp/grammar")
        self.declare_parameter("LLM_WRAPPER_SERVICE", "/nlp/llm")
        self.declare_parameter("COMMON_INTEREST_SERVICE", "/nlp/common_interest")
        self.declare_parameter("IS_POSITIVE_SERVICE", "/nlp/is_positive")
        self.declare_parameter("IS_NEGATIVE_SERVICE", "/nlp/is_negative")
        self.declare_parameter("CATEGORIZE_SERVICE", "/nlp/categorize_shelves")

        self.declare_parameter("temperature", 0.5)
        base_url = self.get_parameter("base_url").get_parameter_value().string_value

        if base_url == "None":
            base_url = None

        self.client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY", "ollama"), base_url=base_url
        )
        self.temperature = (
            self.get_parameter("temperature").get_parameter_value().double_value
        )

        SPEECH_COMMAND_TOPIC = (
            self.get_parameter("SPEECH_COMMAND_TOPIC_NAME")
            .get_parameter_value()
            .string_value
        )

        OUT_COMMAND_TOPIC = (
            self.get_parameter("OUT_COMMAND_TOPIC_NAME")
            .get_parameter_value()
            .string_value
        )

        grammar_service = (
            self.get_parameter("GRAMMAR_SERVICE").get_parameter_value().string_value
        )

        llm_wrapper_service = (
            self.get_parameter("LLM_WRAPPER_SERVICE").get_parameter_value().string_value
        )

        common_interest_service = (
            self.get_parameter("COMMON_INTEREST_SERVICE")
            .get_parameter_value()
            .string_value
        )

        is_positive_service = (
            self.get_parameter("IS_POSITIVE_SERVICE").get_parameter_value().string_value
        )
        is_negative_service = (
            self.get_parameter("IS_NEGATIVE_SERVICE").get_parameter_value().string_value
        )

        categorize_shelves_service = (
            self.get_parameter("CATEGORIZE_SERVICE").get_parameter_value().string_value
        )

        self.create_service(Grammar, grammar_service, self.grammar_service)

        self.create_service(LLMWrapper, llm_wrapper_service, self.llm_wrapper_service)

        self.create_service(
            CommonInterest, common_interest_service, self.common_interest
        )

        self.create_service(IsPositive, is_positive_service, self.is_positive)
        self.create_service(IsNegative, is_negative_service, self.is_negative)

        self.create_service(
            CategorizeShelves, categorize_shelves_service, self.categorize_shelves
        )

        # publisher
        self.publisher = self.create_publisher(Bool, OUT_COMMAND_TOPIC, 10)
        self.logger.info("Initialized llm_utils node")

    def grammar_service(self, req, res):
        response = (
            self.client.beta.chat.completions.parse(
                model=MODEL.GRAMMAR.value,
                temperature=self.temperature,
                messages=[
                    {
                        "role": "system",
                        "content": "You will be presented with some text. Your task is to fix the grammar so that the text is correct. Output ONLY the corrected text, don't include any additional explanations.",
                    },
                    {"role": "user", "content": req.text},
                ],
            )
            .choices[0]
            .message.content
        )

        print("response:", response)

        res.corrected_text = response

        return res

    def llm_wrapper_service(self, req, res):
        context = req.context
        question = req.question

        messages = get_previous_command_answer(context, question)

        response = (
            self.client.beta.chat.completions.parse(
                model=MODEL.LLM_WRAPPER.value,
                temperature=self.temperature,
                messages=messages,
            )
            .choices[0]
            .message.content
        )

        if "</think>" in response:
            response = response.split("</think>")[-1].strip()

        res.answer = response
        return res

    def common_interest(self, req, res):
        self.get_logger().info("Generating common interest")

        messages = get_common_interests_dialog(
            req.person1, req.person2, req.interests1, req.interests2
        )["messages"]
        response = (
            self.client.beta.chat.completions.parse(
                model=MODEL.CommonInterest.value,
                temperature=self.temperature,
                messages=messages,
            )
            .choices[0]
            .message.content
        )

        res.common_interest = response

        return res

    def generic_structured_output(self, messages, response_format):
        self.get_logger().info("Generating structured output")
        # self.get_logger().info(f"System prompt: {system_prompt}")
        # self.get_logger().info(f"User prompt: {user_prompt}")
        # self.get_logger().info(f"Response format: {response_format}")
        response = (
            self.client.beta.chat.completions.parse(
                model=MODEL.GENERIC_STRUCTURED_OUTPUT.value,
                temperature=self.temperature,
                messages=messages,
                response_format=response_format,
            )
            .choices[0]
            .message.content
        )
        self.get_logger().info(f"Response: {response}")
        try:
            response_data = json.loads(response)
            result = response_format(**response_data)
        except Exception as e:
            self.get_logger().error(f"Service error: {e}")
            raise rclpy.exceptions.ServiceException(str(e))
        return result

    def categorize_shelves(
        self, request: CategorizeShelves.Request, response: CategorizeShelves.Response
    ) -> CategorizeShelves.Response:
        """Service to categorize shelves."""

        self.get_logger().info("Categorizing shelves")
        self.get_logger().info("request.shelves: " + str(request.shelves.data))

        shelves: dict[int, list[str]] = eval(request.shelves.data)
        shelves = {int(k): v for k, v in shelves.items()}
        self.get_logger().info(f"Shelves: {shelves}")

        messages, response_format = get_categorize_shelves_args(shelves)

        response_content = (
            self.client.beta.chat.completions.parse(
                model=MODEL.CATEGORIZE_SHELVES.value,
                temperature=self.temperature,
                messages=messages,
                response_format=response_format,
            )
            .choices[0]
            .message.content
        )

        if "</think>" in response_content:
            response_content = response_content.split("</think>")[-1].strip()

        try:
            categorized_shelves = json.loads(response_content)
            self.get_logger().info(f"Categorized shelves: {categorized_shelves}")
            response.categorized_shelves = [
                str(c) for c in categorized_shelves["categories"]
            ]

            return response
        except Exception as e:
            print(f"Error parsing JSON: {e}")
            self.get_logger().error(f"Service error: {e}")
            categorized_shelves = self.generic_structured_output(
                format_response(response_content), response_format
            )
            print(f"Structured response: {categorized_shelves}")

        try:
            response.categorized_shelves = [
                str(c) for c in categorized_shelves.categories
            ]
        except Exception as e:
            self.get_logger().error(f"Service error: {e}")
            raise rclpy.exceptions.ServiceException(str(e))

        self.get_logger().info(f"Response: {str(response.objects_to_add)}")
        self.get_logger().info(
            f"Categorized shelves: {str(response.categorized_shelves)}"
        )

        return response

    def is_positive(
        self, request: IsPositive.Request, response: IsPositive.Response
    ) -> IsPositive.Response:
        """Service to extract information from text."""

        self.get_logger().info("Determining if text is positive")
        messages, response_format = get_is_answer_positive_args(request.text)

        response_content = (
            self.client.beta.chat.completions.parse(
                model=MODEL.IS_POSITIVE.value,
                temperature=self.temperature,
                messages=messages,
                response_format=response_format,
            )
            .choices[0]
            .message.content
        )

        self.get_logger().info(f"The text is: {response_content}")

        try:
            response_data = json.loads(response_content)
            result = response_format(**response_data)
        except Exception as e:
            self.get_logger().error(f"Service error: {e}")
            raise rclpy.exceptions.ServiceException(str(e))

        response.is_positive = result.is_positive
        return response

    def is_negative(
        self, request: IsNegative.Request, response: IsNegative.Response
    ) -> IsNegative.Response:
        """Service to extract information from text."""

        self.get_logger().info("Determining if text is negative")
        messages, response_format = get_is_answer_negative_args(request.text)

        response_content = (
            self.client.beta.chat.completions.parse(
                model=MODEL.IS_NEGATIVE.value,
                temperature=self.temperature,
                messages=messages,
                response_format=response_format,
            )
            .choices[0]
            .message.content
        )

        self.get_logger().info(f"The text is: {response_content}")

        try:
            response_data = json.loads(response_content)
            result = response_format(**response_data)
        except Exception as e:
            self.get_logger().error(f"Service error: {e}")
            raise rclpy.exceptions.ServiceException(str(e))

        response.is_negative = result.is_negative
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(LLMUtils())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
