#!/usr/bin/env python3

import pathlib
import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_TOPIC,
    DISHWASHER_LAYOUT_DETECTION_TOPIC,
    DISHWASHER_RACK_DETECTION_TOPIC,
)
from frida_interfaces.msg import ObjectDetection, ObjectDetectionArray
from frida_interfaces.srv import DishwasherDetection
from geometry_msgs.msg import PointStamped
from vision_general.utils.calculations import (
    get_depth,
    deproject_pixel_to_point,
    get2DCentroid,
)

class DishwasherNode(Node):
    def __init__(self):
        super().__init__("dishwasher")
        self.get_logger().info("Dishwasher node initialized")

        MODELS_PATH = pathlib.Path(__file__).resolve().parent.parent / "Utils" / "models"
        
        self.layout_model = YOLO(str(MODELS_PATH / "dishwasher_layout.pt"))
        self.get_logger().info("Dishwasher layout model loaded")

        self.rack_model = YOLO(str(MODELS_PATH / "dishwasher_rack.pt"))
        self.get_logger().info("Rack model loaded")

        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.image_info_subscriber = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, 10
        )

        self.image_depth_subscriber = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.image_depth_callback, 10
        )

        self.dishwasher_detection_service = self.create_service(
            DishwasherDetection,
            DISHWASHER_LAYOUT_DETECTION_TOPIC,
            self.dishwasher_layout_callback,
            callback_group=self.callback_group,
        )

        self.rack_detection_service = self.create_service(
            DishwasherDetection,
            DISHWASHER_RACK_DETECTION_TOPIC,
            self.dishwasher_rack_callback,
            callback_group=self.callback_group,
        )
        
        self.image = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def image_info_callback(self, msg):
        self.image_info = msg

    def image_depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def dishwasher_layout_callback(self, request, response):
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

    def dishwasher_rack_callback(self, request, response):
        if self.image is None or self.depth_image is None or self.image_info is None:
            response.detection_array = ObjectDetectionArray()
            response.detection_array.detections = []
            response.success = False
            return response

        results = self.rack_model(
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

                point_2D = get2DCentroid(
                    [x1, y1, x2, y2],
                    self.image,
                )

                depth = get_depth(self.depth_image, point_2D)
                point_3D_ = deproject_pixel_to_point(
                    self.image_info, point_2D, depth
                )
                
                point_3D = PointStamped()
                point_3D.header.frame_id = CAMERA_FRAME
                point_3D.header.stamp = self.get_clock().now().to_msg()

                point_3D.point.x = float(point_3D_[0])
                point_3D.point.y = float(point_3D_[1])
                point_3D.point.z = float(point_3D_[2])

                object_detection = ObjectDetection(
                    label=class_id,
                    label_text=class_name,
                    score=confidence,
                    xmin=x1,
                    ymin=y1,
                    xmax=x2,
                    ymax=y2,
                    point3d=point_3D,
                )
                
                response.detection_array.detections.append(object_detection)

        response.success = True
        return response

if __name__ == "__main__":
    rclpy.init()
    dishwasher_node = DishwasherNode()
    rclpy.spin(dishwasher_node)
    rclpy.shutdown()

