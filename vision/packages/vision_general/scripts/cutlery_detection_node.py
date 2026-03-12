#!/usr/bin/env python3
"""
Node to detect cutlery objects (knife, spoons and forks).
"""

import pathlib
import rclpy
import cv2
from sensor_msgs.msg import Image
from frida_interfaces.msg import Detection
from frida_interfaces.srv import YoloDetect
from rclpy.node import Node
from cv_bridge import CvBridge
from ultralytics import YOLO
from frida_constants.vision_constants import CAMERA_TOPIC
from ament_index_python.packages import get_package_share_directory

FORK_CLASS = 42
KNIFE_CLASS = 43
SPOON_CLASS = 44
CONF_THRESHOLD = 0.2


class CutleryDetectionNode(Node):
    def __init__(self):
        super().__init__("cutlery_detection_node")
        self.get_logger().info("Cutlery detection node initialized")
        self.bridge = CvBridge()
        self.image = None

        MODELS_PATH = (
            pathlib.Path(get_package_share_directory("vision_general"))
            / "Utils"
            / "models"
        )

        self.cutlery_model = YOLO(str(MODELS_PATH / "cutlery.pt"))
        self.get_logger().info("Cutlery model loaded")

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.cutlery_detections_publisher = self.create_publisher(
            Image, "cutlery_detections", 5
        )

        self.detect_service = self.create_service(
            YoloDetect, "cutlery_detector", self.detect_callback
        )

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.publish_cutlery_annotations()

    def detect_callback(self, req, res):
        """Run YOLO on the latest cached frame."""
        if self.image is None:
            self.get_logger().warn("No image received yet from camera.")
            res.success = False
            res.detections = "[]"
            self.get_logger().info(
                f"YOLO res: success={res.success}, detections={res.detections}"
            )
            return res

        results = self.cutlery_model(self.image, verbose=False, classes=[0, 1, 3])

        detections = []
        annotated = self.image.copy()
        for out in results:
            for box in out.boxes:
                conf = box.conf.item()
                if conf < CONF_THRESHOLD:
                    continue
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                cls_id = int(box.cls.item())

                det = Detection()
                det.x1 = x1
                det.y1 = y1
                det.x2 = x2
                det.y2 = y2
                det.confidence = conf
                det.class_id = cls_id

                detections.append(det)
                label = f"{self.cutlery_model.names[cls_id]}: {conf:.2f}"
                color = (0, 255, 0)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    annotated,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

        # Publish annotated image to topic
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        self.cutlery_detections_publisher.publish(annotated_msg)

        res.success = True
        res.detections = detections
        self.get_logger().info(
            f"YOLO res: success={res.success}, detections={[(d.class_id, d.confidence) for d in detections]}"
        )
        return res

    def publish_cutlery_annotations(self):
        """Detect cutlery and publish annotated image"""
        if self.image is None:
            return

        results = self.cutlery_model(self.image, verbose=False, classes=[0, 1, 3])
        annotated = self.image.copy()
        for res in results:
            for box in res.boxes:
                conf = box.conf.item()
                if conf < CONF_THRESHOLD:
                    continue
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                cls_id = int(box.cls.item())
                label = f"{self.cutlery_model.names[cls_id]}: {conf:.2f}"
                self.get_logger().info(f"{label} at ({x1}, {y1}), ({x2}, {y2})")
                color = (0, 255, 0)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    annotated,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        self.cutlery_detections_publisher.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CutleryDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
