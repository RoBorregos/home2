"""
End-of-experiment survey shown on the touch display (SurveyOverlay in display_ui.py).

The task manager publishes the questions as JSON on DISPLAY_SURVEY_TOPIC and waits for
the participant's answers on DISPLAY_SURVEY_ANSWERS_TOPIC.
"""

import json
import time
import uuid

import rclpy
from frida_constants.hri_constants import (
    DISPLAY_SURVEY_ANSWERS_TOPIC,
    DISPLAY_SURVEY_TOPIC,
)
from std_msgs.msg import String

MODES = ("display", "auto", "skip")


class SurveyRunner:
    """mode: display (touch screen), auto (scripted answers, for tests) or skip."""

    def __init__(self, node, survey_path: str, mode: str = "display", timeout: float = 180.0):
        if mode not in MODES:
            raise ValueError(f"Survey mode must be one of {MODES}")
        with open(survey_path, "r") as file:
            self.survey = json.load(file)
        self.node = node
        self.mode = mode
        self.timeout = timeout
        self._answers = None
        self._expected_id = None
        self._publisher = node.create_publisher(String, DISPLAY_SURVEY_TOPIC, 10)
        node.create_subscription(String, DISPLAY_SURVEY_ANSWERS_TOPIC, self._on_answers, 10)

    @property
    def intro(self) -> str:
        return self.survey.get(
            "intro", "Please answer a short survey on my screen. It takes one minute."
        )

    def _on_answers(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get("id") == self._expected_id:
            self._answers = data

    def run(self, scripted_answers: dict | None = None) -> dict | None:
        if self.mode == "skip":
            return None
        question_ids = [q["id"] for q in self.survey["questions"]]
        if self.mode == "auto":
            answers = dict(scripted_answers or {})
            return {
                "completed": True,
                "mode": "auto",
                "answers": answers,
                "skipped": [q for q in question_ids if q not in answers],
                "duration_s": 0.0,
            }

        self._expected_id = uuid.uuid4().hex[:8]
        self._answers = None
        payload = {**self.survey, "id": self._expected_id}
        self._publisher.publish(String(data=json.dumps(payload)))

        start = time.monotonic()
        last_publish = start
        while self._answers is None and time.monotonic() - start < self.timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # Re-send in case the display started after the first message
            if time.monotonic() - last_publish > 5.0:
                self._publisher.publish(String(data=json.dumps(payload)))
                last_publish = time.monotonic()

        if self._answers is None:
            self._publisher.publish(String(data=""))  # close the overlay
            return {
                "completed": False,
                "mode": "display",
                "answers": {},
                "skipped": question_ids,
                "duration_s": round(time.monotonic() - start, 1),
            }
        return {**self._answers, "mode": "display"}
