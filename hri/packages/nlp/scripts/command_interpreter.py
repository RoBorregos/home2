#!/usr/bin/env python3

"""
This ROS 2 node interprets the commands received from the Speech processing,
using the fine-tuned model to send the actions to the Task Manager
"""

import json
import os
from typing import List, Optional

import rclpy
from nlp.assets.prompts import get_system_prompt_ci_v2
from openai import OpenAI
from pydantic import BaseModel, Field
from rclpy.node import Node
from std_msgs.msg import String

from frida_interfaces.msg import Command, CommandList


class CommandShape(BaseModel):
    action: str = Field(description="The action to be performed")
    characteristic: Optional[str] = Field(
        description="A characteristic related to the action"
    )
    complement: Optional[str] = Field(description="A complement related to the action")


class CommandListShape(BaseModel):
    commands: List[CommandShape]


class CommandInterpreter(Node):
    def __init__(self):
        """Initialize the ROS 2 node, subscribers and publishers"""
        super().__init__("command_interpreter")

        # Declare and get parameters
        self.declare_parameter("base_url", "None")
        self.declare_parameter("model", "gpt-4o-2024-08-06")
        self.declare_parameter("speech_command_topic", "/speech/raw_command")
        self.declare_parameter("out_command_topic", "/task_manager/commands")

        base_url = self.get_parameter("base_url").get_parameter_value().string_value
        model = self.get_parameter("model").get_parameter_value().string_value
        speech_command_topic = (
            self.get_parameter("speech_command_topic")
            .get_parameter_value()
            .string_value
        )
        out_command_topic = (
            self.get_parameter("out_command_topic").get_parameter_value().string_value
        )

        if base_url == "None":
            base_url = None

        self.model = model
        self.client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY", "ollama"), base_url=base_url
        )

        # Subscribers and publishers
        self.subscriber = self.create_subscription(
            String, speech_command_topic, self._callback, 10
        )
        self.publisher = self.create_publisher(CommandList, out_command_topic, 10)

        self.get_logger().info("Initialized Command Interpreter")

    def _callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        print("Received command: ", data.data)
        self.run(data.data)

    def run(self, raw_command: str) -> None:
        """Method for running the interpretation of the commands"""
        response = (
            self.client.beta.chat.completions.parse(
                model=self.model,
                messages=[
                    {"role": "system", "content": get_system_prompt_ci_v2()},
                    {"role": "user", "content": raw_command},
                ],
                response_format=CommandListShape,
            )
            .choices[0]
            .message.content
        )

        try:
            response_data = json.loads(response)
            result = CommandListShape(**response_data)
        except Exception as e:
            self.get_logger().error(f"Service error: {e}")
            return

        self.get_logger().debug(f"Commands interpreted: {result.commands}")

        command_list = CommandList()
        command_list.commands = [
            Command(
                action=command.action,
                characteristic=command.characteristic,
                complement=command.complement,
            )
            for command in result.commands
        ]
        self.publisher.publish(command_list)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(CommandInterpreter())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger("CommandInterpreter").error(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
