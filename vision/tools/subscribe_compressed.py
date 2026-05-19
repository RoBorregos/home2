#!/usr/bin/env python3
"""Simple ROS2 node to subscribe to a CompressedImage topic and display/save frames.

Usage:
  python3 vision/tools/subscribe_compressed.py --topic /vision/detections_image/compressed
"""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CompressedViewer(Node):
    def __init__(self, topic: str):
        super().__init__("compressed_viewer")
        self.sub = self.create_subscription(CompressedImage, topic, self.cb, 10)
        self.last = time.time()
        self.count = 0

    def cb(self, msg: CompressedImage):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warning("Received empty/compressed image decode failed")
                return
            cv2.imshow("compressed_view", img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rclpy.shutdown()
            self.count += 1
            if self.count % 30 == 0:
                now = time.time()
                fps = 30.0 / (now - self.last)
                self.get_logger().info(f"FPS (avg recent): {fps:.1f}")
                self.last = now
        except Exception as e:
            self.get_logger().error(f"Error decoding compressed image: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/vision/detections_image/compressed")
    args = parser.parse_args()

    rclpy.init()
    node = CompressedViewer(args.topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
