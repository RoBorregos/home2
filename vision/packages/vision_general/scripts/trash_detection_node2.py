#!/usr/bin/env python3
"""
Node exposing a service to filter YOLO detections by label_text.
"""

import rclpy
from rclpy.node import Node

from frida_constants.vision_constants import DETECTIONS_TOPIC, TRASH_SERVICE
from frida_interfaces.msg import ObjectDetectionArray
from frida_interfaces.srv import DetectionHandler


class TrashDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("trash_detection_node")
        self.latest_detections = ObjectDetectionArray()

        self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self.detections_callback,
            10,
        )
        self.create_service(
            DetectionHandler,
            TRASH_SERVICE,
            self.get_trash,
        )

        self.get_logger().info("Trash service ready")

    def detections_callback(self, data) -> None:
        self.latest_detections = data

    def get_trash(self, req, response):
        if not self.latest_detections.detections:
            response.detection_array = ObjectDetectionArray(detections=[])
            response.success = False
            self.get_logger().warn("No detections available yet")
            return response

        requested_label = req.label.strip()
        requested_labels = [label.strip() for label in req.labels if label.strip()]

        if requested_label:
            filtered = [
                det
                for det in self.latest_detections.detections
                if det.label_text.lower() == requested_label.lower()
            ]
        elif requested_labels:
            allowed_labels = {label.lower() for label in requested_labels}
            filtered = [
                det
                for det in self.latest_detections.detections
                if det.label_text.lower() in allowed_labels
            ]
        else:
            filtered = list(self.latest_detections.detections)

        # debug
        for det in filtered:
            self.get_logger().info(
                f"Found {det.label_text} at ({det.xmin}, {det.ymin}), ({det.xmax}, {det.ymax})"
            )

        response.detection_array = ObjectDetectionArray(detections=filtered)
        response.success = len(filtered) > 0
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrashDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
