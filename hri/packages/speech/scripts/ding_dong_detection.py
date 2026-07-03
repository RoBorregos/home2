#!/usr/bin/env python3
"""
Ding-dong (chime doorbell) detection node — DSP detector (no Edge Impulse).

Thin ROS wrapper around ``speech.ding_dong_detection_utils.DingDongDetector``.
Publishes ``{"keyword": "doorbell", "score": <float>}`` on the same topic as the
knock and EI door nodes so the task manager needs only one subscription
(``_get_door_event`` -> ``door_event_detected``). Pure DSP and self-calibrating,
so it runs in every environment (unlike the EI nodes, which are l4t only) and
needs no per-doorbell tuning.
"""

import json

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from frida_interfaces.msg import AudioData
from speech.ding_dong_detection_utils import DingDongDetector, DingDongDetectorConfig


class DingDongDetectionNode(Node):
    def __init__(self):
        super().__init__("ding_dong_detection")

        self.declare_parameter("audio_topic", "/hri/rawAudioChunk")
        self.declare_parameter("KEYWORD_TOPIC", "/hri/speech/ei_detection")
        self.declare_parameter("doorbell_label", "doorbell")
        self.declare_parameter("sample_rate", 16000)
        # Only publish a ding-dong scoring above this value.
        self.declare_parameter("min_score", 0.55)
        # Stage 1 — adaptive loudness gate (no fixed min_db)
        self.declare_parameter("active_margin_db", 8.0)
        self.declare_parameter("floor_alpha", 0.05)
        # Stage 2 — pitch + clarity (scale-free)
        self.declare_parameter("pitch_min_hz", 200.0)
        self.declare_parameter("pitch_max_hz", 4000.0)
        self.declare_parameter("clarity_min", 0.6)
        # Stage 3 — note grouping
        self.declare_parameter("pitch_tol", 0.06)
        self.declare_parameter("note_gap_ms", 120.0)
        self.declare_parameter("min_note_ms", 60.0)
        self.declare_parameter("max_note_ms", 1600.0)
        # Stage 4 — temporal + pitch pattern
        self.declare_parameter("min_tones", 2)
        self.declare_parameter("min_gap_s", 0.03)
        self.declare_parameter("max_gap_s", 1.50)
        self.declare_parameter("pattern_window_s", 3.5)
        self.declare_parameter("require_descending", True)
        self.declare_parameter("min_drop_ratio", 0.03)
        self.declare_parameter("cooldown_s", 1.5)

        def g(name):
            return self.get_parameter(name).value

        audio_topic = g("audio_topic")
        result_topic = g("KEYWORD_TOPIC")
        self.doorbell_label = g("doorbell_label")
        self.sample_rate = g("sample_rate")
        self.min_score = g("min_score")

        cfg = DingDongDetectorConfig(
            sample_rate=self.sample_rate,
            active_margin_db=g("active_margin_db"),
            floor_alpha=g("floor_alpha"),
            pitch_min_hz=g("pitch_min_hz"),
            pitch_max_hz=g("pitch_max_hz"),
            clarity_min=g("clarity_min"),
            pitch_tol=g("pitch_tol"),
            note_gap_ms=g("note_gap_ms"),
            min_note_ms=g("min_note_ms"),
            max_note_ms=g("max_note_ms"),
            min_tones=g("min_tones"),
            min_gap_s=g("min_gap_s"),
            max_gap_s=g("max_gap_s"),
            pattern_window_s=g("pattern_window_s"),
            require_descending=g("require_descending"),
            min_drop_ratio=g("min_drop_ratio"),
            cooldown_s=g("cooldown_s"),
        )
        self.detector = DingDongDetector(cfg)

        self.publisher = self.create_publisher(String, result_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)

        self.get_logger().info(
            f"DingDongDetectionNode ready | in: {audio_topic} | out: {result_topic} | "
            f"tones: {cfg.min_tones} | descending: {cfg.require_descending} | "
            f"self-calibrating (margin {cfg.active_margin_db} dB)"
        )

    def audio_callback(self, msg):
        chunk = np.frombuffer(bytes(msg.data), dtype=np.int16)
        for event in self.detector.process(chunk):
            notes = ", ".join(f"{t.freq_hz:.0f}Hz" for t in event.tones)
            if event.score < self.min_score:
                self.get_logger().info(
                    f"Ding-dong below threshold ({event.score:.2f} < "
                    f"{self.min_score:.2f}) | notes=[{notes}] — ignored"
                )
                continue
            self.get_logger().info(
                f"Detection: {self.doorbell_label} ({event.score:.2f}) | "
                f"notes=[{notes}] span={event.span_s:.2f}s"
            )
            detection_info = {
                "keyword": self.doorbell_label,
                "score": float(event.score),
            }
            self.publisher.publish(String(data=json.dumps(detection_info)))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(DingDongDetectionNode())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
