#!/usr/bin/env python3

import pathlib
import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from frida_constants.vision_constants import (
    CAMERA_TOPIC, 
    DISHWASHER_LAYOUT_DETECTION_TOPIC
)
from frida_interfaces.msg import ObjectDetection, ObjectDetectionArray
from frida_interfaces.srv import DishwasherDetection

class DishwasherNode(Node):
    def __init__(self):
        super().__init__("dishwasher")
        self.get_logger().info("Dishwasher node initialized")

        MODELS_PATH = {
            pathlib.Path(__file__).resolve().parent.parent / "Utils" / "models"
        }
        
        self.layout_model = YOLO(str(MODELS_PATH / "dishwasher.pt"))
        self.get_logger().info("Dishwasher layout model loaded")

        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.dishwasher_detection_service = self.create_service(
            DishwasherDetection,
            DISHWASHER_LAYOUT_DETECTION_TOPIC,
            self.detect_dishwasher_callback,
            callback_group=self.callback_group,
        )

        self.image = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def detect_dishwasher_callback(self, request, response):
        if self.image is None:
            response.detection_array = ObjectDetectionArray()
            response.detection_array.detections = []
            response.success = False
            return response

        results = self.layout_model(
            source=self.image,
            conf=0.35,
            verbose=False,
        )

        response.detection_array = ObjectDetectionArray()
        response.detection_array.detections = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                class_name = result.names.get(class_id, str(class_id))
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                confidence = float(box.conf[0].item())

                object_detection = ObjectDetection(
                    label=class_id,
                    label_text=class_name,
                    score=confidence,
                    xmin=x1,
                    ymin=y1,
                    xmax=x2,
                    ymax=y2,
                )
                response.detection_array.detections.append(object_detection)

        response.success = True
        return response

if __name__ == "__main__":
    rclpy.init()
    dishwasher_node = DishwasherNode()
    rclpy.spin(dishwasher_node)
    rclpy.shutdown()

