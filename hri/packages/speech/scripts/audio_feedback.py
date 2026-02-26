#!/usr/bin/env python3
import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CURRENT_FILE_PATH = os.path.abspath(__file__)


def find_assets_dir():
    current = os.path.dirname(CURRENT_FILE_PATH)
    while current != "/":
        potential_assets = os.path.join(current, "assets")
        if os.path.exists(potential_assets) and os.path.isdir(potential_assets):
            return potential_assets
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return None


ASSETS_DIR = find_assets_dir()


class AudioFeedbackNode(Node):
    def __init__(self):
        super().__init__("audio_feedback")
        self.get_logger().info("Initializing Audio Feedback node.")

        self.create_subscription(String, "/AudioState", self.audio_state_callback, 10)

        if ASSETS_DIR:
            self.chime_path = os.path.join(ASSETS_DIR, "listening_chime.wav")
        else:
            self.chime_path = None
            self.get_logger().error("Assets directory not found!")

        if self.chime_path and not os.path.exists(self.chime_path):
            self.get_logger().warn(f"Chime file not found at {self.chime_path}")

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
