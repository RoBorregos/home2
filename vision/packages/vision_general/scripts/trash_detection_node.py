#!/usr/bin/env python3
"""
Node exposing a service to filter YOLO detections by label_text.
"""

import rclpy
from rclpy.node import Node

from frida_constants.vision_constants import DETECTIONS_TOPIC, TRASH_SERVICE_CLASSES
from frida_interfaces.msg import ObjectDetectionArray, ObjectDetection
from frida_interfaces.srv import SetDetectorClasses

DEFAULT_LABELS = [
    "apple",
    "milk",
    "juice",
]


class TrashDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("trash_detection_node")
        self.latest_detections = ObjectDetectionArray()
        self.labels = DEFAULT_LABELS

        self.create_subscription(
            ObjectDetectionArray,
            DETECTIONS_TOPIC,
            self.detections_callback,
            10,
        )

        self.create_service(
            SetDetectorClasses,
            TRASH_SERVICE_CLASSES,
            self.set_trash_classes,
        )

        self.trash_pub = self.create_publisher(
            ObjectDetectionArray, DETECTIONS_TOPIC, 10
        )

        self.get_logger().info("Trash service ready")

    def detections_callback(self, data) -> None:
        self.latest_detections = data
        self.publish_trash()

    def set_trash_classes(self, req, res):
        if not req.class_names or len(req.class_names) == 0:
            res.success = False
            self.get_logger().warn("No labels input in request")
            return res
        elif req.class_names:
            requested_labels = [label.strip() for label in req.class_names]
            allowed_labels = {label.lower() for label in requested_labels}
        else:
            allowed_labels = {}

        self.labels = allowed_labels
        res.success = True
        return res

    def publish_trash(self):
        filtered = ObjectDetectionArray()

        for det in self.latest_detections.detections:
            label = det.label_text.strip().lower()
            if not label or label.startswith("trash/"):
                continue

            detection = ObjectDetection()
            if det.label_text.lower() in self.labels:
                self.get_logger().info(f"Detected TRASH/{det.label_text}")
                detection.label = det.label
                detection.label_text = f"trash/{det.label_text}"
                detection.score = det.score
                detection.ymin = det.ymin
                detection.xmin = det.xmin
                detection.ymax = det.ymax
                detection.xmax = det.xmax
                detection.point3d = det.point3d
                filtered.detections.append(detection)

        self.trash_pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = TrashDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
