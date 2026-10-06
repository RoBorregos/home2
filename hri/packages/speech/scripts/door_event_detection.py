#!/usr/bin/env python3
"""
Door-event detection node — doorbell and knock via a pretrained AudioSet tagger.

Thin ROS wrapper around ``speech.door_event_tagger.DoorEventTagger``. Publishes
``{"keyword": "doorbell" | "knock", "score": <float>}`` on the shared door-event
topic (``/hri/speech/ei_detection``), the same contract the task manager already
consumes.

Gating is unchanged from the DSP doorbell node: it only listens while *armed*
(the task manager arms it via ``arm_topic`` while waiting at the door) and while
the robot is not speaking (``/saying``). Inference only runs while listening.
"""

import json
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from frida_interfaces.msg import AudioData
from speech.door_event_tagger import (
    DEFAULT_DOORBELL_CLASSES,
    DEFAULT_KNOCK_CLASSES,
    DoorEventTagger,
    DoorEventTaggerConfig,
)
from speech.efficientat import WEIGHTS_FILE


class DoorEventDetectionNode(Node):
    def __init__(self):
        super().__init__("door_event_detection")

        self.declare_parameter("audio_topic", "/hri/rawAudioChunk")
        self.declare_parameter("KEYWORD_TOPIC", "/hri/speech/ei_detection")
        self.declare_parameter("sample_rate", 16000)
        # Gating.
        self.declare_parameter("arm_topic", "/hri/doorbell/armed")
        self.declare_parameter("saying_topic", "/saying")
        # If True, nothing is tagged or published until an arm=True message is
        # received. Set False to run always (e.g. standalone debugging).
        self.declare_parameter("require_arm", True)
        self.declare_parameter("mute_while_speaking", True)
        # Model. Relative paths resolve against the package root, like
        # noise_cancellation's DF_MODEL_PATH.
        self.declare_parameter(
            "weights_path", os.path.join("assets/downloads/efficientat", WEIGHTS_FILE)
        )
        self.declare_parameter("device", "")
        # Detection.
        self.declare_parameter("window_s", 5.0)
        self.declare_parameter("hop_s", 0.5)
        self.declare_parameter("threshold", 0.15)
        self.declare_parameter("strong_threshold", 0.5)
        self.declare_parameter("min_consecutive", 2)
        self.declare_parameter("cooldown_s", 2.0)
        self.declare_parameter("min_db", -55.0)
        self.declare_parameter("doorbell_classes", DEFAULT_DOORBELL_CLASSES)
        self.declare_parameter("knock_classes", DEFAULT_KNOCK_CLASSES)
        self.declare_parameter("doorbell_keyword", "doorbell")
        self.declare_parameter("knock_keyword", "knock")

        def g(name):
            return self.get_parameter(name).value

        weights_path = g("weights_path")
        if not os.path.isabs(weights_path):
            script_dir = os.path.dirname(os.path.realpath(__file__))
            weights_path = os.path.abspath(os.path.join(script_dir, "..", weights_path))

        self.require_arm = g("require_arm")
        self.mute_while_speaking = g("mute_while_speaking")
        self._armed = not self.require_arm
        self._speaking = False
        self._was_listening = False

        cfg = DoorEventTaggerConfig(
            sample_rate=g("sample_rate"),
            window_s=g("window_s"),
            hop_s=g("hop_s"),
            threshold=g("threshold"),
            strong_threshold=g("strong_threshold"),
            min_consecutive=g("min_consecutive"),
            cooldown_s=g("cooldown_s"),
            min_db=g("min_db"),
            doorbell_classes=list(g("doorbell_classes")),
            knock_classes=list(g("knock_classes")),
            doorbell_keyword=g("doorbell_keyword"),
            knock_keyword=g("knock_keyword"),
            weights_path=weights_path,
            device=g("device"),
        )
        self.detector = DoorEventTagger(cfg)

        self.publisher = self.create_publisher(String, g("KEYWORD_TOPIC"), 10)
        self.create_subscription(AudioData, g("audio_topic"), self.audio_callback, 10)
        self.create_subscription(Bool, g("arm_topic"), self._arm_callback, 10)
        if self.mute_while_speaking:
            self.create_subscription(Bool, g("saying_topic"), self._saying_callback, 10)

        self.get_logger().info("DoorEventDetection ready")
        self.get_logger().debug(
            f"DoorEventDetection | in: {g('audio_topic')} | out: {g('KEYWORD_TOPIC')} | "
            f"device: {self.detector.tagger.device} | require_arm: {self.require_arm} | "
            f"threshold: {cfg.threshold:.2f} x{cfg.min_consecutive} "
            f"(strong {cfg.strong_threshold:.2f})"
        )

    # ── gating ───────────────────────────────────────────────────────────────

    def _arm_callback(self, msg: Bool) -> None:
        if msg.data == self._armed:
            return
        self._armed = msg.data
        self.get_logger().info(f"Door detection {'ARMED' if msg.data else 'disarmed'}")

    def _saying_callback(self, msg: Bool) -> None:
        self._speaking = msg.data

    @property
    def _listening(self) -> bool:
        return self._armed and not (self.mute_while_speaking and self._speaking)

    # ── audio ────────────────────────────────────────────────────────────────

    def audio_callback(self, msg: AudioData) -> None:
        listening = self._listening
        if listening and not self._was_listening:
            # Start from a clean buffer so audio heard while disarmed or while
            # the robot spoke (TTS, listening chime) can't leak into a window.
            self.detector.reset()
        self._was_listening = listening
        if not listening:
            return

        chunk = np.frombuffer(bytes(msg.data), dtype=np.int16)
        for event in self.detector.process(chunk):
            self.get_logger().info(
                f"Detection: {event.keyword} ({event.label} p={event.score:.2f}, "
                f"speech p={event.speech_score:.2f})"
            )
            self.publisher.publish(
                String(
                    data=json.dumps({"keyword": event.keyword, "score": event.score})
                )
            )

        scores = self.detector.last_scores
        if scores is not None:
            self.detector.last_scores = None
            keyword, label, score, speech = scores
            self.get_logger().debug(
                f"window: best {label or '-'} p={score:.2f} | speech p={speech:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(DoorEventDetectionNode())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
