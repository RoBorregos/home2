#!/usr/bin/env python3
"""
Doorbell detection node — robust DSP detector.

Thin ROS wrapper around ``speech.doorbell_detection_utils.DoorbellDetector``.
Publishes ``{"keyword": "doorbell", "score": <float>}`` on the shared door-event
topic (``/hri/speech/ei_detection``) so the task manager needs only one
subscription. Self-calibrating and round-independent, so it runs everywhere.

Adds two things around the raw detector:
  * Gating — only publishes while *armed* (the task manager arms it via
    ``arm_topic`` while waiting at the door) and while not speaking (muted via
    ``/saying``), which is where the specificity comes from.
  * Auto-enrollment — the first confident ring is enrolled as a spectral
    template so a later, fainter ring of the same round's bell still matches.
"""

import json

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from frida_interfaces.msg import AudioData
from speech.doorbell_detection_utils import DoorbellDetector, DoorbellDetectorConfig


class DoorbellDetectionNode(Node):
    def __init__(self):
        super().__init__("doorbell_detection")

        self.declare_parameter("audio_topic", "/hri/rawAudioChunk")
        self.declare_parameter("KEYWORD_TOPIC", "/hri/speech/ei_detection")
        self.declare_parameter("doorbell_label", "doorbell")
        self.declare_parameter("sample_rate", 16000)
        # Only publish a ring scoring above this value.
        self.declare_parameter("min_score", 0.6)
        # Gating.
        self.declare_parameter("arm_topic", "/hri/doorbell/armed")
        self.declare_parameter("saying_topic", "/saying")
        # If True, publish nothing until an arm=True message is received. The task
        # manager arms only while waiting at the door. Set False to run always
        # (e.g. standalone debugging).
        self.declare_parameter("require_arm", True)
        self.declare_parameter("mute_while_speaking", True)
        # Auto-enrollment (round-independent template matching).
        self.declare_parameter("enroll", True)
        # Detector config — event confirmation is by loudness, not tonal shape.
        self.declare_parameter("active_margin_db", 8.0)
        self.declare_parameter("confirm_margin_db", 12.0)
        self.declare_parameter("floor_alpha", 0.05)
        self.declare_parameter("event_gap_ms", 150.0)
        self.declare_parameter("min_event_ms", 120.0)
        self.declare_parameter("max_event_ms", 4000.0)
        self.declare_parameter("cooldown_s", 1.5)
        # Tonal features below feed the score/fingerprint only; they never reject.
        self.declare_parameter("pitch_min_hz", 200.0)
        self.declare_parameter("pitch_max_hz", 1600.0)
        self.declare_parameter("clarity_min", 0.60)
        self.declare_parameter("template_sim_confirm", 0.85)

        def g(name):
            return self.get_parameter(name).value

        audio_topic = g("audio_topic")
        result_topic = g("KEYWORD_TOPIC")
        self.doorbell_label = g("doorbell_label")
        self.sample_rate = g("sample_rate")
        self.min_score = g("min_score")
        self.require_arm = g("require_arm")
        self.mute_while_speaking = g("mute_while_speaking")
        self.enroll = g("enroll")
        self.template_sim_confirm = g("template_sim_confirm")

        # Armed unless we are told to wait for an explicit arm signal.
        self._armed = not self.require_arm
        self._speaking = False

        cfg = DoorbellDetectorConfig(
            sample_rate=self.sample_rate,
            active_margin_db=g("active_margin_db"),
            confirm_margin_db=g("confirm_margin_db"),
            floor_alpha=g("floor_alpha"),
            event_gap_ms=g("event_gap_ms"),
            min_event_ms=g("min_event_ms"),
            max_event_ms=g("max_event_ms"),
            cooldown_s=g("cooldown_s"),
            pitch_min_hz=g("pitch_min_hz"),
            pitch_max_hz=g("pitch_max_hz"),
            clarity_min=g("clarity_min"),
            template_sim_confirm=self.template_sim_confirm,
        )
        self.detector = DoorbellDetector(cfg)

        self.publisher = self.create_publisher(String, result_topic, 10)
        self.create_subscription(AudioData, audio_topic, self.audio_callback, 10)
        self.create_subscription(Bool, g("arm_topic"), self._arm_callback, 10)
        if self.mute_while_speaking:
            self.create_subscription(Bool, g("saying_topic"), self._saying_callback, 10)

        self.get_logger().info(
            f"DoorbellDetectionNode ready | in: {audio_topic} | out: {result_topic} | "
            f"require_arm: {self.require_arm} | enroll: {self.enroll} | "
            f"loud-event (confirm >={cfg.confirm_margin_db:.0f} dB over floor, "
            f">={cfg.min_event_ms:.0f} ms)"
        )

    # ── gating ───────────────────────────────────────────────────────────────

    def _arm_callback(self, msg: Bool) -> None:
        if msg.data == self._armed:
            return
        self._armed = msg.data
        self.get_logger().info(
            f"Doorbell detection {'ARMED' if msg.data else 'disarmed'}"
        )

    def _saying_callback(self, msg: Bool) -> None:
        self._speaking = msg.data

    @property
    def _listening(self) -> bool:
        return self._armed and not (self.mute_while_speaking and self._speaking)

    # ── audio ────────────────────────────────────────────────────────────────

    def audio_callback(self, msg):
        chunk = np.frombuffer(bytes(msg.data), dtype=np.int16)

        # High-pass filter to reject low-frequency loud sounds (thuds, slamming doors)
        float_chunk = chunk.astype(np.float64)
        if len(float_chunk) > 0:
            if hasattr(self, "_last_audio_sample"):
                last_val = self._last_audio_sample
            else:
                last_val = 0.0
            filtered = np.empty_like(float_chunk)
            filtered[0] = float_chunk[0] - 0.85 * last_val
            filtered[1:] = float_chunk[1:] - 0.85 * float_chunk[:-1]
            self._last_audio_sample = float_chunk[-1]
        else:
            filtered = float_chunk

        events = self.detector.process(filtered)

        if not self._listening:
            return

        for event in events:
            # Reject explicitly low-pitch sounds that still passed the energy threshold
            if 0 < event.dominant_hz < 400.0:
                self.get_logger().info(
                    f"Rejected low-frequency sound: {event.dominant_hz:.0f}Hz"
                )
                continue

            info = f"{event.margin_db:.0f}dB/{event.dur_ms:.0f}ms/{event.dominant_hz:.0f}Hz"
            sim = event.template_similarity
            # Confirm if the raw score passes, or if the fingerprint clearly
            # matches this round's enrolled bell (recovers quieter/degraded rings).
            confirmed = event.score >= self.min_score or (
                self.detector.has_template and sim >= self.template_sim_confirm
            )
            if not confirmed:
                self.get_logger().info(
                    f"Event below threshold (score={event.score:.2f} < "
                    f"{self.min_score:.2f}, sim={sim:.2f}) | {info} — ignored"
                )
                continue

            self.get_logger().info(
                f"Detection: {self.doorbell_label} (score={event.score:.2f}, "
                f"sim={sim:.2f}) | {info}"
            )
            # Enroll this round's bell from the first confident ring so later
            # rings match by template even if fainter.
            if (
                self.enroll
                and not self.detector.has_template
                and event.spectrum is not None
            ):
                self.detector.enroll_template(event.spectrum)
                self.get_logger().info(
                    "Enrolled this ring as the round's doorbell template."
                )

            detection_info = {
                "keyword": self.doorbell_label,
                "score": float(event.score),
            }
            self.publisher.publish(String(data=json.dumps(detection_info)))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(DoorbellDetectionNode())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
