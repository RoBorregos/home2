#!/usr/bin/env python3
"""Scripted HRI: stands in for the speech stack so no mic, speakers or LLM are needed.

Speak requests are logged to the transcript, republished on /speech/text_spoken and
held for as long as a real voice would take, in sim time. When the scenario has people,
they answer the robot through the speech-to-text action and the extraction services.
"""

import json
import os
import re

import rclpy
from frida_constants.hri_constants import (
    EXTRACT_DATA_SERVICE,
    LLM_WRAPPER_SERVICE,
    SPEAK_SERVICE,
    STT_ACTION_SERVER_NAME,
)
from frida_interfaces.action import SpeechStream
from frida_interfaces.srv import ExtractInfo, LLMWrapper, Speak
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from frida_task_sim import scenario as scenarios

TEXT_SPOKEN_TOPIC = "/speech/text_spoken"
# Speaking time per character at speed 1.0 (s)
SECONDS_PER_CHAR = 0.06
# Pause before a person answers (s)
REPLY_DELAY = 1.0
# What follows these words in an answer is the value the robot asked for
VALUE_RE = re.compile(r"(?:\bis|\bto the) (.+?)\.?$", re.IGNORECASE)


class FakeHri(Node):
    def __init__(self):
        super().__init__("fake_hri")
        run_dir = self.declare_parameter("run_dir", "").value
        self.transcript = os.path.join(run_dir, "transcript.jsonl") if run_dir else ""
        self.spoken_pub = self.create_publisher(String, TEXT_SPOKEN_TOPIC, 10)
        self.create_service(
            Speak, SPEAK_SERVICE, self._speak, callback_group=ReentrantCallbackGroup()
        )
        scenario_file = self.declare_parameter("scenario_file", "").value
        self.people = scenarios.load(scenario_file).get("people", [])
        self.drop = scenarios.load(scenario_file).get("drop", {})
        self.last_said = ""
        self.greeted = 0
        if self.people:
            group = ReentrantCallbackGroup()
            ActionServer(
                self,
                SpeechStream,
                STT_ACTION_SERVER_NAME,
                self._listen,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=group,
            )
            self.create_service(
                ExtractInfo, EXTRACT_DATA_SERVICE, self._extract, callback_group=group
            )
            self.create_service(
                LLMWrapper, LLM_WRAPPER_SERVICE, self._llm, callback_group=group
            )
        self.get_logger().info("Scripted HRI ready")

    def sleep(self, seconds: float):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            self.get_clock().sleep_for(Duration(seconds=0.05))

    def reply(self) -> tuple[str, str, float]:
        """(speaker, text, confidence) answering the last thing the robot said."""
        guests = [p for p in self.people if p["id"].startswith("guest")]
        guest = guests[min(self.greeted, len(guests) - 1)]
        said = self.last_said.lower()
        if "where would you like me to go" in said:
            return "host", f"Please go to the {self.drop['location']}", 0.95
        if "what is your name" in said:
            return guest["id"], f"My name is {guest['name']}", guest["stt_confidence"]
        if "favorite drink" in said:
            return (
                guest["id"],
                f"My favorite drink is {guest['drink']}",
                guest["stt_confidence"],
            )
        if said.rstrip().endswith("?"):
            return ("host" if self.greeted >= len(guests) else guest["id"]), "yes", 0.95
        return "", "", 0.0

    def _listen(self, goal_handle):
        who, text, confidence = self.reply()
        result = SpeechStream.Result()
        self.sleep(REPLY_DELAY if text else goal_handle.request.start_silence_time)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result
        if text:
            self.log(who, text)
            goal_handle.publish_feedback(
                SpeechStream.Feedback(current_transcription=text)
            )
            self.sleep(0.5)
            result.transcription = text
            result.words = text.split()
            result.confidences = [confidence] * len(result.words)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result

    def _extract(self, request, response):
        match = VALUE_RE.search(request.full_text.strip())
        response.result = match.group(1) if match else ""
        return response

    def _llm(self, request, response):
        response.answer = f"They are {request.context.strip(' ;')}."
        return response

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
        self.last_said = request.text
        self.greeted += request.text.startswith("Nice to meet you")
        self.sleep(len(request.text) * SECONDS_PER_CHAR / max(request.speed, 0.1))
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
