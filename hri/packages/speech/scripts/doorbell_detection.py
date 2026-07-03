#!/usr/bin/env python3
"""
Doorbell (chime) detection node — robust DSP detector.

Thin ROS wrapper around ``speech.doorbell_detection_utils.DoorbellDetector``.
Publishes ``{"keyword": "doorbell", "score": <float>}`` on the shared door-event
topic (``/hri/speech/ei_detection``) so the task manager needs only one
subscription. Round-independent (no per-doorbell tuning) and self-calibrating,
so it runs in every environment (unlike the EI door node, which is l4t only).

Two robustness features beyond the raw detector:

  * Gating — the detector only publishes while *armed* and while the robot is
    *not speaking*. The task manager arms it (``arm_topic``, std_msgs/Bool) only
    while it is parked at the start position waiting for a guest, which is the
    only moment the doorbell rings; the robot's own TTS is muted via ``/saying``.
    This removes almost all false positives from party speech and self-audio.
  * Auto-enrollment — the doorbell is the same within a round (it rings for both
    guests). The first confident ring is enrolled as a spectral template so the
    second guest's ring is recognised with higher confidence, even if the bell
    sound changes between rounds.
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
        # Detector config.
        self.declare_parameter("active_margin_db", 10.0)
        self.declare_parameter("floor_alpha", 0.05)
        self.declare_parameter("pitch_min_hz", 200.0)
        self.declare_parameter("pitch_max_hz", 4000.0)
        self.declare_parameter("clarity_min", 0.80)
        self.declare_parameter("pitch_tol", 0.06)
        self.declare_parameter("note_gap_ms", 80.0)
        self.declare_parameter("sustain_min_ms", 140.0)
        self.declare_parameter("max_note_ms", 2500.0)
        self.declare_parameter("min_decay_ratio", 1.25)
        self.declare_parameter("min_tones", 1)
        self.declare_parameter("min_gap_s", 0.03)
        self.declare_parameter("max_gap_s", 2.0)
        self.declare_parameter("pattern_window_s", 3.5)
        self.declare_parameter("cooldown_s", 1.5)
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
            floor_alpha=g("floor_alpha"),
            pitch_min_hz=g("pitch_min_hz"),
            pitch_max_hz=g("pitch_max_hz"),
            clarity_min=g("clarity_min"),
            pitch_tol=g("pitch_tol"),
            note_gap_ms=g("note_gap_ms"),
            sustain_min_ms=g("sustain_min_ms"),
            max_note_ms=g("max_note_ms"),
            min_decay_ratio=g("min_decay_ratio"),
            min_tones=g("min_tones"),
            min_gap_s=g("min_gap_s"),
            max_gap_s=g("max_gap_s"),
            pattern_window_s=g("pattern_window_s"),
            cooldown_s=g("cooldown_s"),
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
            f"self-calibrating (margin {cfg.active_margin_db} dB, "
            f"sustain>={cfg.sustain_min_ms:.0f}ms)"
        )

    # ── gating ───────────────────────────────────────────────────────────────

    def _arm_callback(self, msg: Bool) -> None:
        if msg.data == self._armed:
            return
        self._armed = msg.data
        # Re-calibrate the ambient floor and drop any half-built note whenever we
        # (dis)arm, so a stale note can't leak across the boundary.
        self.detector.reset()
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
        if not self._listening:
            return
        chunk = np.frombuffer(bytes(msg.data), dtype=np.int16)
        for event in self.detector.process(chunk):
            notes = ", ".join(f"{t.freq_hz:.0f}Hz" for t in event.tones)
            sim = event.template_similarity
            # Confirm if the raw score passes, or if the fingerprint clearly
            # matches this round's enrolled bell (recovers quieter/degraded rings).
            confirmed = event.score >= self.min_score or (
                self.detector.has_template and sim >= self.template_sim_confirm
            )
            if not confirmed:
                self.get_logger().info(
                    f"Ring below threshold (score={event.score:.2f} < "
                    f"{self.min_score:.2f}, sim={sim:.2f}) | notes=[{notes}] — ignored"
                )
                continue

            self.get_logger().info(
                f"Detection: {self.doorbell_label} (score={event.score:.2f}, "
                f"sim={sim:.2f}) | notes=[{notes}] span={event.span_s:.2f}s"
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
