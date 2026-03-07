#!/usr/bin/env python3

"""
Node to initialize and provide a YOLO instance for reuse across other files.
"""

import pathlib
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from frida_constants.vision_constants import CAMERA_TOPIC
from frida_interfaces.srv import YoloDetect
from frida_interfaces.msg import Detection

import cv2

YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/Utils/yolo26n.pt"
CONF_THRESHOLD = 0.5


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.bridge = CvBridge()
        self.latest_frame = None

        # Load YOLO once
        self.get_logger().info(f"Loading YOLO model from {YOLO_LOCATION}...")
        self.model = YOLO(YOLO_LOCATION)
        self.get_logger().info("YOLO model loaded successfully")

        self.detect_service = self.create_service(
            YoloDetect, "yolo_detect", self.detect_callback
        )

        # Subscribe to camera
        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        # Publisher for annotated image
        self.detections_image_publisher = self.create_publisher(
            Image, "yolo_detections_image", 5
        )

    def image_callback(self, msg: Image):
        """Cache the latest image from the camera and call detection/publish function."""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        self.publish_annotated_image()

    def publish_annotated_image(self):
        """Run YOLO detection and publish annotated image."""
        if self.latest_frame is None:
            return
        classes = None
        results = self.model(self.latest_frame, verbose=False, classes=classes)
        annotated = self.latest_frame.copy()
        for out in results:
            for box in out.boxes:
                conf = box.conf.item()
                if conf < CONF_THRESHOLD:
                    continue
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                cls_id = int(box.cls.item())
                label = f"{self.model.names[cls_id]}: {conf:.2f}"
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
        self.detections_image_publisher.publish(annotated_msg)

    def detect_callback(self, request, response):
        """Run YOLO on the latest cached frame."""
        if self.latest_frame is None:
            self.get_logger().warn("No image received yet from camera.")
            response.success = False
            response.detections = "[]"
            self.get_logger().info(
                f"YOLO response: success={response.success}, detections={response.detections}"
            )
            return response

        # If caller provided class IDs, use them; otherwise detect all
        classes = list(request.classes) if request.classes else None
        print(f"Running YOLO detection with classes: {classes if classes else 'all'}")
        results = self.model(self.latest_frame, verbose=False, classes=classes)

        detections = []
        annotated = self.latest_frame.copy()
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
                label = f"{self.model.names[cls_id]}: {conf:.2f}"
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
        self.detections_image_publisher.publish(annotated_msg)

        response.success = True
        response.detections = detections
        self.get_logger().info(
            f"YOLO response: success={response.success}, detections={[(d.class_id, d.confidence) for d in detections]}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
