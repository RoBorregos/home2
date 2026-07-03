#!/usr/bin/env python3
"""
Knock detection node — DSP detector (no Edge Impulse).

Thin ROS wrapper around ``speech.knock_detection_utils.KnockDetector``. Publishes
``{"keyword": "knock", "score": <float>}`` on the same topic as the EI door node
so the task manager needs only one subscription. Pure DSP, so it runs in every
environment (unlike the EI nodes, which are l4t only).
"""

import json

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from frida_interfaces.msg import AudioData
from speech.knock_detection_utils import KnockDetector, KnockDetectorConfig


class KnockDetectionNode(Node):
    def __init__(self):
        super().__init__("knock_detection")

        self.declare_parameter("audio_topic", "/hri/rawAudioChunk")
        self.declare_parameter("KEYWORD_TOPIC", "/hri/speech/ei_detection")
        self.declare_parameter("knock_label", "knock")
        self.declare_parameter("sample_rate", 16000)
        # Only publish a knock scoring above this value.
        self.declare_parameter("min_score", 0.55)
        # Gating: a knock is only a door signal while the robot waits at the door,
        # and its own audio must not be mistaken for one. The task manager arms
        # the detector; /saying mutes it while the robot speaks.
        self.declare_parameter("arm_topic", "/hri/doorbell/armed")
        self.declare_parameter("saying_topic", "/saying")
        self.declare_parameter("require_arm", True)
        self.declare_parameter("mute_while_speaking", True)
        # filtro 1: energy / dB gate
        self.declare_parameter("min_db", -40.0)
        # Onset detection
        self.declare_parameter("onset_ratio", 3.0)
        self.declare_parameter("use_bandpass", True)
        self.declare_parameter("bandpass_low_hz", 60.0)
        self.declare_parameter("bandpass_high_hz", 1000.0)
        self.declare_parameter("refractory_s", 0.08)
        # Temporal pattern
        self.declare_parameter("min_onsets", 2)
        self.declare_parameter("pattern_window_s", 1.5)
        self.declare_parameter("min_onset_gap_s", 0.10)
        self.declare_parameter("cooldown_s", 1.0)

        def g(name):
            return self.get_parameter(name).value

        audio_topic = g("audio_topic")
        result_topic = g("KEYWORD_TOPIC")
        self.knock_label = g("knock_label")
        self.sample_rate = g("sample_rate")
        self.min_score = g("min_score")

        cfg = KnockDetectorConfig(
            sample_rate=self.sample_rate,
            min_db=g("min_db"),
            onset_ratio=g("onset_ratio"),
            use_bandpass=g("use_bandpass"),
            bandpass_low_hz=g("bandpass_low_hz"),
            bandpass_high_hz=g("bandpass_high_hz"),
            refractory_s=g("refractory_s"),
            min_onsets=g("min_onsets"),
            pattern_window_s=g("pattern_window_s"),
            min_onset_gap_s=g("min_onset_gap_s"),
            cooldown_s=g("cooldown_s"),
        )
        self.detector = KnockDetector(cfg)

        self.require_arm = g("require_arm")
        self.mute_while_speaking = g("mute_while_speaking")
        self._armed = not self.require_arm
        self._speaking = False

        self.publisher = self.create_publisher(String, result_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)
        self.create_subscription(Bool, g("arm_topic"), self._arm_callback, 10)
        if self.mute_while_speaking:
            self.create_subscription(Bool, g("saying_topic"), self._saying_callback, 10)

        self.get_logger().info(
            f"KnockDetectionNode ready | in: {audio_topic} | out: {result_topic} | "
            f"min_db: {cfg.min_db} dBFS | min_onsets: {cfg.min_onsets} | "
            f"require_arm: {self.require_arm}"
        )

    def _arm_callback(self, msg):
        if msg.data == self._armed:
            return
        self._armed = msg.data
        # Drop any half-built burst so a stale onset can't leak across the boundary.
        self.detector.reset()
        self.get_logger().info(f"Knock detection {'ARMED' if msg.data else 'disarmed'}")

    def _saying_callback(self, msg):
        self._speaking = msg.data

    @property
    def _listening(self):
        return self._armed and not (self.mute_while_speaking and self._speaking)

    def audio_callback(self, msg):
        if not self._listening:
            return
        chunk = np.frombuffer(bytes(msg.data), dtype=np.int16)
        for event in self.detector.process(chunk):
            if event.score < self.min_score:
                self.get_logger().info(
                    f"Knock below threshold ({event.score:.2f} < {self.min_score:.2f}) "
                    f"| onsets={event.num_onsets} peak={event.peak_db:.1f} dBFS — ignored"
                )
                continue
            self.get_logger().info(
                f"Detection: {self.knock_label} ({event.score:.2f}) | "
                f"onsets={event.num_onsets} peak={event.peak_db:.1f} dBFS"
            )
            detection_info = {
                "keyword": self.knock_label,
                "score": float(event.score),
            }
            self.publisher.publish(String(data=json.dumps(detection_info)))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(KnockDetectionNode())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
