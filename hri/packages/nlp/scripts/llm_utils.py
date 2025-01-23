#!/usr/bin/env python3

"""Miscellanous functions that interact with an LLM."""

import json
import os
from typing import Optional

import rclpy
from openai import OpenAI
from pydantic import BaseModel
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from frida_interfaces.srv import Grammar

SPEECH_COMMAND_TOPIC = "/speech/raw_command"
OUT_COMMAND_TOPIC = "/stop_following"

exit_keys = ["exit", "quit", "stop", "end", "terminate", "finish", "cancel"]


class ResponseFormat(BaseModel):
    is_stop: bool


class LLMUtils(Node):
    model: Optional[str]
    base_url: Optional[str]

    def __init__(self) -> None:
        global SPEECH_COMMAND_TOPIC, OUT_COMMAND_TOPIC
        super().__init__("llm_utils")
        self.logger = self.get_logger()
        self.logger.info("Initializing llm_utils node")

        self.declare_parameter("base_url", "None")
        self.declare_parameter("model", "gpt-4o-2024-08-06")
        self.declare_parameter("SPEECH_COMMAND_TOPIC_NAME", SPEECH_COMMAND_TOPIC)
        self.declare_parameter("OUT_COMMAND_TOPIC_NAME", OUT_COMMAND_TOPIC)
        self.declare_parameter("GRAMMAR_SERVICE", "/nlp/grammar")

        self.declare_parameter("temperature", 0.5)
        base_url = self.get_parameter("base_url").get_parameter_value().string_value

        if base_url == "None":
            self.base_url = None
        else:
            self.base_url = base_url

        model = self.get_parameter("model").get_parameter_value().string_value
        self.model = model
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

        self.create_service(Grammar, grammar_service, self.grammar_service)

        # publisher
        self.publisher = self.create_publisher(Bool, OUT_COMMAND_TOPIC, 10)
        self.subscription = self.create_subscription(
            String, SPEECH_COMMAND_TOPIC, self.callback, 10
        )
        self.logger.info("Initialized llm_utils node")

    def callback(self, data: String) -> None:
        if data.data == "" or len(data.data) == 0:
            self.logger.debug("Empty command received")
            return

        for key in exit_keys:
            if key in data.data.lower():
                self.logger.info("Stop command received")
                msg = Bool()
                msg.data = True
                self.publisher.publish(msg)
                return

        response = self.client.beta.chat.completions.parse(
            model=self.model,
            temperature=self.temperature,
            messages=[
                {
                    "role": "system",
                    "content": f"Please determine if the command is a stop command, here is a sample of stop commands: {exit_keys}",
                },
                {"role": "user", "content": data.data},
            ],
            response_format=ResponseFormat,
        )

        response = response.choices[0].message.content

        parse = json.loads(response)

        parsed = ResponseFormat(**parse)

        if parsed.is_stop:
            self.logger.info("Stop command received")
            msg = Bool()
            msg.data = True
            self.publisher.publish(msg)
        return

    def grammar_service(self, req, res):
        response = (
            self.client.beta.chat.completions.parse(
                model=self.model,
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
