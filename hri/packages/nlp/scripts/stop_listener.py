from pydantic import BaseModel
import openai
import os
import rclpy
import rclpy.impl.rcutils_logger
from std_msgs.msg import String, Bool
from typing import Optional
import json

SPEECH_COMMAND_TOPIC = "/speech/raw_command"
OUT_COMMAND_TOPIC = "/stop_following"

exit_keys = ["exit", "quit", "stop", "end", "terminate", "finish", "cancel"]


class ResponseFormat(BaseModel):
    is_stop: bool


class StopListenerNode(rclpy.node.Node):
    logger: rclpy.impl.rcutils_logger.RcutilsLogger
    model: Optional[str]
    base_url: Optional[str]

    def __init__(self) -> None:
        global SPEECH_COMMAND_TOPIC, OUT_COMMAND_TOPIC
        super().__init__("stop_listener")

        self.declare_parameter("base_url", None)
        self.declare_parameter("model", "gpt-4o-2024-08-06")
        self.declare_parameter("SPEECH_COMMAND_TOPIC_NAME", SPEECH_COMMAND_TOPIC)
        self.declare_parameter("OUT_COMMAND_TOPIC_NAME", OUT_COMMAND_TOPIC)

        base_url = self.get_parameter("base_url").get_parameter_value().string_value

        if base_url == "None":
            self.base_url = None
        else:
            self.base_url = base_url

        model = self.get_parameter("model").get_parameter_value().string_value
        self.model = model

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

        self.logger = self.get_logger()
        self.logger.info("Starting stop listener node")

        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.logger.info("Stop listener node started")

        # publisher
        self.publisher = self.create_publisher(Bool, OUT_COMMAND_TOPIC, 10)
        self.subscription = self.create_subscription(
            String, SPEECH_COMMAND_TOPIC, self.callback, 10
        )

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

        response = openai.beta.chat.completions.parse(
            model=self.model,
            base_url=self.base_url,
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


def main(args=None):
    rclpy.init(args=args)
    node = StopListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()
