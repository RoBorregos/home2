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

YOLO_LOCATION = str(pathlib.Path(__file__).parent) + "/Utils/yolov8n.pt"
CONF_THRESHOLD = 0.5


class YoloServer(Node):
    def __init__(self):
        super().__init__("yolo_server")
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

    def image_callback(self, msg: Image):
        """Cache the latest image from the camera."""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def detect_callback(self, request, response):
        """Run YOLO on the latest cached frame."""
        if self.latest_frame is None:
            self.get_logger().warn("No image received yet from camera.")
            response.success = False
            response.detections = "[]"
            return response

        # If caller provided class IDs, use them; otherwise detect all
        classes = list(request.classes) if request.classes else None
        results = self.model(self.latest_frame, verbose=False, classes=classes)

        detections = []
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

        response.success = True
        response.detections = detections

        self.get_logger().info(f"Detected {len(detections)} objects")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
