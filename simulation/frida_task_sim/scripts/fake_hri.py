#!/usr/bin/env python3
"""Scripted HRI: stands in for the speech stack so no mic, speakers or LLM are needed.

Speak requests are logged to the transcript, republished on /speech/text_spoken and
held for as long as a real voice would take, in sim time.
"""

import json
import os

import rclpy
from frida_constants.hri_constants import SPEAK_SERVICE
from frida_interfaces.srv import Speak
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

TEXT_SPOKEN_TOPIC = "/speech/text_spoken"
# Speaking time per character at speed 1.0 (s)
SECONDS_PER_CHAR = 0.06


class FakeHri(Node):
    def __init__(self):
        super().__init__("fake_hri")
        run_dir = self.declare_parameter("run_dir", "").value
        self.transcript = os.path.join(run_dir, "transcript.jsonl") if run_dir else ""
        self.spoken_pub = self.create_publisher(String, TEXT_SPOKEN_TOPIC, 10)
        self.create_service(
            Speak, SPEAK_SERVICE, self._speak, callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Scripted HRI ready")

    def log(self, who: str, text: str):
        self.spoken_pub.publish(String(data=json.dumps({"who": who, "text": text})))
        if self.transcript:
            row = {
                "t": self.get_clock().now().nanoseconds / 1e9,
                "who": who,
                "text": text,
            }
            with open(self.transcript, "a") as f:
                f.write(json.dumps(row) + "\n")

    def _speak(self, request, response):
        self.log("robot", request.text)
        duration = len(request.text) * SECONDS_PER_CHAR / max(request.speed, 0.1)
        end = self.get_clock().now() + Duration(seconds=duration)
        while rclpy.ok() and self.get_clock().now() < end:
            self.get_clock().sleep_for(Duration(seconds=0.05))
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FakeHri()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
