#!/usr/bin/env python3
import os
import subprocess
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String
from ament_index_python.packages import get_package_share_directory

from frida_constants.hri_constants import SPEAKER_TEST_TOPIC

SPEAKER_TEST_CHIMES = 3


class AudioFeedbackNode(Node):
    def __init__(self):
        super().__init__("audio_feedback")
        self.get_logger().debug("Initializing Audio Feedback node.")

        self.create_subscription(String, "/AudioState", self.audio_state_callback, 10)
        self.create_subscription(
            Empty, SPEAKER_TEST_TOPIC, self.speaker_test_callback, 10
        )
        self._speaker_test_running = threading.Event()

        try:
            share_dir = get_package_share_directory("speech")
            self.chime_path = os.path.join(share_dir, "assets", "listening_chime.wav")
        except Exception:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.chime_path = os.path.join(
                current_dir, "..", "assets", "listening_chime.wav"
            )

        if not os.path.exists(self.chime_path):
            self.get_logger().error(f"Chime file not found at {self.chime_path}")

        self.get_logger().info("AudioFeedback ready")

    def audio_state_callback(self, msg):
        """Play sound when state changes to listening."""
        if msg.data == "listening":
            if self.chime_path and os.path.exists(self.chime_path):
                self.get_logger().info("Playing listening chime.")
                subprocess.Popen(["aplay", "-q", self.chime_path])
            else:
                self.get_logger().error(
                    f"Cannot play chime: file not found at {self.chime_path}"
                )

    def speaker_test_callback(self, _msg):
        """Play the chime several times so the speaker volume can be checked."""
        if not (self.chime_path and os.path.exists(self.chime_path)):
            self.get_logger().error(
                f"Cannot play chime: file not found at {self.chime_path}"
            )
            return
        if self._speaker_test_running.is_set():
            return
        self._speaker_test_running.set()
        # aplay blocks until the chime ends; keep the executor free meanwhile.
        threading.Thread(target=self._play_speaker_test, daemon=True).start()

    def _play_speaker_test(self):
        try:
            self.get_logger().info("Speaker test: playing chimes.")
            for _ in range(SPEAKER_TEST_CHIMES):
                subprocess.run(["aplay", "-q", self.chime_path], check=False)
        finally:
            self._speaker_test_running.clear()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AudioFeedbackNode()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
