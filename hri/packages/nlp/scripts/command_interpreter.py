#!/usr/bin/env python3

"""
This ROS 2 node interprets the commands received from the Speech processing,
using the fine-tuned model to send the actions to the Task Manager
"""

import json
import os

import rclpy
from nlp.assets.prompts import get_system_prompt_expo
from nlp.assets.schemas import CommandListShape
from openai import OpenAI
from rclpy.node import Node
from std_msgs.msg import String

from frida_constants.hri_constants import MODEL
from frida_interfaces.msg import Command, CommandList
from frida_interfaces.srv import CommandInterpreter as CommandInterpreterSrv


class CommandInterpreter(Node):
    def __init__(self):
        """Initialize the ROS 2 node, subscribers and publishers"""
        super().__init__("command_interpreter")

        # Declare and get parameters
        self.declare_parameter("base_url", "None")
        self.declare_parameter("speech_command_topic", "/speech/raw_command")
        self.declare_parameter("publish_command_topic", "/task_manager/commands")
        self.declare_parameter(
            "COMMAND_INTERPRETER_SERVICE", "/nlp/command_interpreter"
        )

        self.declare_parameter("temperature", 0.5)

        base_url = self.get_parameter("base_url").get_parameter_value().string_value
        speech_command_topic = (
            self.get_parameter("speech_command_topic")
            .get_parameter_value()
            .string_value
        )
        publish_command_topic = (
            self.get_parameter("publish_command_topic")
            .get_parameter_value()
            .string_value
        )
        command_interpreter_service = (
            self.get_parameter("COMMAND_INTERPRETER_SERVICE")
            .get_parameter_value()
            .string_value
        )

        self.temperature = (
            self.get_parameter("temperature").get_parameter_value().double_value
        )

        if base_url == "None":
            base_url = None

        self.client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY", "ollama"), base_url=base_url
        )

        # Subscribers and publishers
        self.subscriber = self.create_subscription(
            String, speech_command_topic, self._callback, 10
        )
        self.publisher = self.create_publisher(CommandList, publish_command_topic, 10)
        self.create_service(
            CommandInterpreterSrv, command_interpreter_service, self.command_service
        )

        self.get_logger().info("Initialized Command Interpreter")

    def _callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        print("Received command: ", data.data)
        self.run(data.data)

    def run(self, raw_command: str) -> None:
        """Method for running the interpretation of the commands"""
        commands = self.get_commands(raw_command)
        self.publisher.publish(commands)

    def command_service(self, req, res):
        commands = self.get_commands(req.text)
        res.commands = commands.commands
        return res

    def get_commands(self, raw_command: str):
        response = (
            self.client.beta.chat.completions.parse(
                model=MODEL.GET_COMMANDS.value,
                temperature=self.temperature,
                messages=[
                    {"role": "system", "content": get_system_prompt_expo()},
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
        return command_list


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
