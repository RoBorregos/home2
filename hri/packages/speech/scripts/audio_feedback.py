#!/usr/bin/env python3
import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class AudioFeedbackNode(Node):
    def __init__(self):
        super().__init__("audio_feedback")
        self.get_logger().info("Initializing Audio Feedback node.")

        self.create_subscription(String, "/AudioState", self.audio_state_callback, 10)

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
        else:
            self.get_logger().info(f"Chime file found: {self.chime_path}")

        self.get_logger().info("Audio Feedback node initialized.")

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


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AudioFeedbackNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
