#!/usr/bin/env python3
"""
Node to detect cutlery objects (knife, spoons and forks).
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from frida_interfaces.srv import YoloDetect

FORK_CLASS = 42
KNIFE_CLASS = 43
SPOON_CLASS = 44


class CutleryDetectionNode(Node):
    def __init__(self):
        super().__init__("cutlery_detection_node")
        self.get_logger().info("Cutlery detection node initialized")
        self.bridge = CvBridge()
        self.image = None

        self.yolo_client = self.create_client(YoloDetect, "yolo_detect")

        while not self.yolo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for yolo_node service to be availble...")

        self.timer = self.create_timer(2.0, self.detect_cutlery)

    def detect_cutlery(self):
        request = YoloDetect.Request()
        request.classes = [FORK_CLASS, KNIFE_CLASS, SPOON_CLASS]

        future = self.yolo_client.call_async(request)
        future.add_done_callback(self.detection_callback)

    def detection_callback(self, future):
        try:
            response = future.result()
            if response.success:
                for det in response.detections:
                    self.get_logger().info(
                        f"bbox=({det.x1}, {det.y1}, {det.x2}, {det.y2})"
                    )
            else:
                self.get_logger().warn("No image available")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CutleryDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
