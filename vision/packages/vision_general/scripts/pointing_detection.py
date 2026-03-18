#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

from frida_interfaces.srv import DetectPointingObject, SetPointingObjectClasses
from frida_interfaces.msg import ObjectDetectionArray
import cv2
import numpy as np
import os
from ultralytics import YOLO
from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    ZERO_SHOT_DETECTIONS_TOPIC,
    POINTING_OBJECT_SERVICE,
    POINTING_DETECTION_IMAGE_TOPIC,
    SET_POINTING_OBJECT_CLASSES_SERVICE,
    CAMERA_FRAME,
)
import threading
import copy

USE_RIGHT_HAND = True
USE_LEFT_HAND = False
INFERENCE_TIMEOUT = 2.0  # seconds
DEFAULT_CLASSES = [
    "handbag",
    "bag",
    "backpack",
    "tote_bag",
]

# YOLO COCO keypoint indices
RIGHT_SHOULDER = 6
RIGHT_WRIST = 10  # closest to mediapipe's RIGHT_HAND (index 20)
LEFT_SHOULDER = 5
LEFT_WRIST = 9    # closest to mediapipe's LEFT_HAND (index 19)
KP_CONF = 0.3


def load_yolo_pose(model_name="yolo11m-pose.pt"):
    """Load YOLO pose model with automatic TensorRT export for Orin AGX."""
    engine_path = model_name.replace(".pt", ".engine")
    if os.path.exists(engine_path):
        return YOLO(engine_path, task="pose")
    model = YOLO(model_name)
    try:
        model.export(format="engine", half=True, device=0, imgsz=640)
        return YOLO(engine_path, task="pose")
    except Exception:
        return model


class DetectPointingObjectServer(Node):
    def __init__(self):
        super().__init__("detect_pointing_object_server")

        self.get_logger().info("Initializing Detect Pointing Object Server")

        self.pose_model = load_yolo_pose("yolo11m-pose.pt")

        self.VISUALIZE = True

        self.objects = []
        self.closest_object = None
        self.detected_objects = []
        self.detected_object_centroids = []

        self.last_inference_time = self.get_clock().now()

        self.bridge = CvBridge()

        qos = rclpy.qos.QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self.detections_sub = self.create_subscription(
            ObjectDetectionArray,
            ZERO_SHOT_DETECTIONS_TOPIC,
            self.detections_callback,
            qos,
        )

        self.image_sub = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos
        )

        self.visualizer_pub = self.create_publisher(
            Image, POINTING_DETECTION_IMAGE_TOPIC, qos
        )

        self._pointing_classes_server = self.create_service(
            SetPointingObjectClasses,
            SET_POINTING_OBJECT_CLASSES_SERVICE,
            self.set_classes_callback,
        )

        self._pointing_object_server = self.create_service(
            DetectPointingObject, POINTING_OBJECT_SERVICE, self.execute_callback
        )

        self.class_names = DEFAULT_CLASSES
        self.active_flag = True
        self.runThread = None

        # Create a timer to check detections lifetime
        self.cleanup_timer = self.create_timer(
            INFERENCE_TIMEOUT,  # seconds
            self.cleanup_detections,
        )

        self.get_logger().info("Detect Pointing Object Server Initialized")

    def cleanup_detections(self):
        current_time = self.get_clock().now()
        if (
            current_time - self.last_inference_time
        ).nanoseconds / 1e9 > INFERENCE_TIMEOUT:
            self.objects = []
            self.closest_object = None

    def execute_callback(self, request, response):
        self.get_logger().info("Detect Pointing Object Server Received Request")

        if self.closest_object is None:
            response.success = False
            self.get_logger().info("No Object Detected")
        else:
            response.success = True
            response.detection = self.closest_object
            self.get_logger().info(
                f"Object detected, sent result: {response.detection.label_text}"
            )

        return response

    def set_classes_callback(self, request, response):
        response = SetPointingObjectClasses.Response()
        self.class_names = request.classes
        response.success = True
        return response

    def detections_callback(self, data):
        self.last_inference_time = self.get_clock().now()
        if (
            len(data.detections) == 0
            and ((self.get_clock().now() - self.last_inference_time).nanoseconds / 1e9)
            > INFERENCE_TIMEOUT
        ):
            self.detected_objects = []
            return
        if len(data.detections) > 0:
            self.detected_objects = []
            j = 0
            for i, detection in enumerate(data.detections):
                if detection.label_text not in self.class_names:
                    continue
                centroid_x = (detection.xmin + detection.xmax) / 2
                centroid_y = (detection.ymin + detection.ymax) / 2
                self.detected_objects.append(
                    {
                        "detection": detection,
                        "centroid": (centroid_x, centroid_y),
                    }
                )
                self.last_inference_time = self.get_clock().now()
                j += 1

    def image_callback(self, data):
        self.bgr_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if self.active_flag and self.runThread is None or not self.runThread.is_alive():
            self.runThread = threading.Thread(
                target=self.run_inference, args=(), daemon=True
            )
            self.runThread.start()

    def run_inference(self):
        self.objects = copy.deepcopy(self.detected_objects)
        visualize_img = self.bgr_img.copy()
        img = self.bgr_img

        # Run YOLO pose instead of mediapipe
        results = self.pose_model(img, verbose=False)
        has_person = (results and results[0].keypoints is not None and
                      results[0].keypoints.xyn is not None and
                      len(results[0].keypoints.xyn) > 0)

        if has_person:
            points = results[0].keypoints.xyn[0].cpu().numpy()  # normalized (0-1)
            conf = (results[0].keypoints.conf[0].cpu().numpy()
                    if results[0].keypoints.conf is not None
                    else np.ones(17, dtype=np.float32))

            # Draw keypoints
            if self.VISUALIZE:
                for idx in [RIGHT_SHOULDER, RIGHT_WRIST, LEFT_SHOULDER, LEFT_WRIST]:
                    if conf[idx] > KP_CONF:
                        px = int(points[idx][0] * img.shape[1])
                        py = int(points[idx][1] * img.shape[0])
                        cv2.circle(visualize_img, (px, py), 5, (0, 255, 0), -1)
                        cv2.putText(visualize_img, str(idx),
                                    (px, py), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (255, 0, 0), 2)

        closest_object = None
        closest_distance = 100000

        if has_person:
            # Right hand pointing
            if (USE_RIGHT_HAND and
                    conf[RIGHT_WRIST] > KP_CONF and conf[RIGHT_SHOULDER] > KP_CONF and
                    self._check_in_bounds(points[RIGHT_WRIST]) and
                    self._check_in_bounds(points[RIGHT_SHOULDER])):
                right_m = ((points[RIGHT_WRIST][1] - points[RIGHT_SHOULDER][1]) /
                           (points[RIGHT_WRIST][0] - points[RIGHT_SHOULDER][0] + 1e-6))
                right_intercept = points[RIGHT_WRIST][1] - right_m * points[RIGHT_WRIST][0]
                right_line_start = (0, int(right_intercept * img.shape[0]))
                right_line_end = (img.shape[1],
                                  int((right_m * 1 + right_intercept) * img.shape[0]))
                cv2.line(visualize_img, right_line_start, right_line_end, (0, 255, 0), 2)
                closest_object, closest_distance, visualize_img = (
                    self.check_closest_object(right_m, right_intercept, visualize_img))

            # Left hand pointing
            if (USE_LEFT_HAND and
                    conf[LEFT_WRIST] > KP_CONF and conf[LEFT_SHOULDER] > KP_CONF and
                    self._check_in_bounds(points[LEFT_WRIST]) and
                    self._check_in_bounds(points[LEFT_SHOULDER])):
                left_m = ((points[LEFT_WRIST][1] - points[LEFT_SHOULDER][1]) /
                          (points[LEFT_WRIST][0] - points[LEFT_SHOULDER][0] + 1e-6))
                left_intercept = points[LEFT_WRIST][1] - left_m * points[LEFT_WRIST][0]
                left_line_start = (0, int(left_intercept * img.shape[0]))
                left_line_end = (img.shape[1],
                                 int((left_m * 1 + left_intercept) * img.shape[0]))
                cv2.line(visualize_img, left_line_start, left_line_end, (0, 255, 0), 2)
                closest_object, closest_distance, visualize_img = (
                    self.check_closest_object(left_m, left_intercept, visualize_img))

            if closest_object is not None:
                marker = Marker()
                marker.header.frame_id = CAMERA_FRAME
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = self.objects[closest_object][
                    "detection"
                ].point3d.point.x
                marker.pose.position.y = self.objects[closest_object][
                    "detection"
                ].point3d.point.y
                marker.pose.position.z = self.objects[closest_object][
                    "detection"
                ].point3d.point.z
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                self.closest_object = self.objects[closest_object]["detection"]

            elif (
                closest_object is None
                and (
                    (self.get_clock().now() - self.last_inference_time).nanoseconds
                    / 1e9
                )
                > INFERENCE_TIMEOUT
            ):
                self.closest_object = None

        # visualize detected objects
        for i, point in enumerate(self.objects):
            color = (0, 255, 0) if i == closest_object else (0, 0, 255)
            if i == closest_object:
                visualize_img = cv2.rectangle(
                    visualize_img,
                    (
                        int(self.objects[i]["detection"].xmin * img.shape[1]),
                        int(self.objects[i]["detection"].ymin * img.shape[0]),
                    ),
                    (
                        int(self.objects[i]["detection"].xmax * img.shape[1]),
                        int(self.objects[i]["detection"].ymax * img.shape[0]),
                    ),
                    color,
                    2,
                )
            else:
                visualize_img = cv2.drawMarker(
                    visualize_img,
                    (
                        int(self.objects[i]["centroid"][0] * img.shape[1]),
                        int(self.objects[i]["centroid"][1] * img.shape[0]),
                    ),
                    color,
                    cv2.MARKER_CROSS,
                    10,
                    2,
                )
                visualize_img = cv2.rectangle(
                    visualize_img,
                    (
                        int(self.objects[i]["detection"].xmin * img.shape[1]),
                        int(self.objects[i]["detection"].ymin * img.shape[0]),
                    ),
                    (
                        int(self.objects[i]["detection"].xmax * img.shape[1]),
                        int(self.objects[i]["detection"].ymax * img.shape[0]),
                    ),
                    color,
                    2,
                )

        img_msg = self.bridge.cv2_to_imgmsg(visualize_img, encoding="bgr8")
        self.visualizer_pub.publish(img_msg)

    def _check_in_bounds(self, point):
        return 0 <= point[0] <= 1 and 0 <= point[1] <= 1

    def check_closest_object(self, m, intercept, visualize_img):
        closest_object = None
        closest_distance = 100000
        for i, object in enumerate(self.objects):
            centroid = object["centroid"]
            distance = abs(centroid[1] - m * centroid[0] - intercept) / np.sqrt(
                m**2 + 1
            )
            if distance < closest_distance:
                closest_distance = distance
                closest_object = i
        return closest_object, closest_distance, visualize_img


def main():
    rclpy.init()
    server = DetectPointingObjectServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
