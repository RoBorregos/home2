#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

# You'll need to create these custom interfaces in your ROS2 package
from frida_interfaces.srv import DetectPointingObject, SetPointingObjectClasses
from frida_interfaces.msg import ObjectDetectionArray
import cv2
import numpy as np
import mediapipe as mp
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
    "bag",
    "hand_bag",
]


class DetectPointingObjectServer(Node):
    def __init__(self):
        super().__init__("detect_pointing_object_server")
        self._default_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.get_logger().info("Initializing Detect Pointing Object Server")

        self.mp_pose = mp.solutions.pose

        self.pose = self.mp_pose.Pose()
        self.VISUALIZE = True
        self.RIGHT_SHOULDER = 12
        self.RIGHT_HAND = 20
        self.LEFT_SHOULDER = 11
        self.LEFT_HAND = 19

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
        """Set the classes for the object detector.

        Args:
            request (SetDetectorClasses.Request): Request object containing the classes.

        Returns:
            SetDetectorClasses.Response: Response object.
        """
        response = SetPointingObjectClasses.Response()
        self.class_names = request.classes
        response.success = True
        return response

    def detections_callback(self, data):
        self.last_inference_time = self.get_clock().now()
        # get the detections
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
        self.bgr_img.flags.writeable = False
        results = self.pose.process(self.bgr_img)

        img = self.bgr_img

        # draw points
        ONLY_SHOULDER_AND_HANDS = True
        if results.pose_landmarks:
            for i, point in enumerate(results.pose_landmarks.landmark):
                if self.VISUALIZE and (
                    not ONLY_SHOULDER_AND_HANDS
                    or i == self.RIGHT_SHOULDER
                    or i == self.RIGHT_HAND
                    or i == self.LEFT_SHOULDER
                    or i == self.LEFT_HAND
                ):
                    visualize_img = cv2.circle(
                        visualize_img,
                        (int(point.x * img.shape[1]), int(point.y * img.shape[0])),
                        5,
                        (0, 255, 0),
                        -1,
                    )
                    cv2.putText(
                        visualize_img,
                        str(i),
                        (int(point.x * img.shape[1]), int(point.y * img.shape[0])),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2,
                    )

        closest_object = None
        closest_distance = 100000

        # get angle between finger tip and hand base
        if results.pose_landmarks:
            finger_tip_left = None
            hand_base_left = None
            finger_tip_right = None
            hand_base_right = None

            for i, point in enumerate(results.pose_landmarks.landmark):
                if i == self.RIGHT_HAND and self.check_visible(point, img):
                    finger_tip_right = point
                if i == self.RIGHT_SHOULDER and self.check_visible(point, img):
                    hand_base_right = point
                if i == self.LEFT_HAND and self.check_visible(point, img):
                    finger_tip_left = point
                if i == self.LEFT_SHOULDER and self.check_visible(point, img):
                    hand_base_left = point

            if (
                finger_tip_right is not None and hand_base_right is not None
            ) and USE_RIGHT_HAND:
                # print(f"Right Hand: {finger_tip_right.x}, {finger_tip_right.y}")
                # print(f"Right Shoulder: {hand_base_right.x}, {hand_base_right.y}")
                right_m = (finger_tip_right.y - hand_base_right.y) / (
                    finger_tip_right.x - hand_base_right.x
                )
                right_intercept = finger_tip_right.y - right_m * finger_tip_right.x
                right_line_start = (0, int(right_intercept * img.shape[0]))
                right_line_end = (
                    img.shape[1],
                    int((right_m * 1 + right_intercept) * img.shape[0]),
                )
                cv2.line(
                    visualize_img, right_line_start, right_line_end, (0, 255, 0), 2
                )
                closest_object, closest_distance, visualize_img = (
                    self.check_closest_object(right_m, right_intercept, visualize_img)
                )
            if (
                finger_tip_left is not None and hand_base_left is not None
            ) and USE_LEFT_HAND:
                left_m = (finger_tip_left.y - hand_base_left.y) / (
                    finger_tip_left.x - hand_base_left.x
                )
                # print(f"Left Hand: {finger_tip_left.x}, {finger_tip_left.y}")
                # print(f"Left Shoulder: {hand_base_left.x}, {hand_base_left.y}")
                left_intercept = finger_tip_left.y - left_m * finger_tip_left.x
                left_line_start = (0, int(left_intercept * img.shape[0]))
                left_line_end = (
                    img.shape[1],
                    int((left_m * 1 + left_intercept) * img.shape[0]),
                )
                cv2.line(visualize_img, left_line_start, left_line_end, (0, 255, 0), 2)
                closest_object, closest_distance, visualize_img = (
                    self.check_closest_object(left_m, left_intercept, visualize_img)
                )

            if closest_object is not None:
                # publish marker
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
                # self.pointed_object_marker.publish(marker)

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

        # visualize points
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

    def check_visible(self, point, img):
        if point.x < 0 or point.x > 1 or point.y < 0 or point.y > 1:
            return False
        return True

    def check_closest_object(self, m, intercept, visualize_img):
        closest_object = None
        closest_distance = 100000
        for i, object in enumerate(self.objects):
            centroid = object["centroid"]
            # visualize_img = self.draw_orthogonal_distance(m, intercept, centroid, visualize_img)
            distance = abs(centroid[1] - m * centroid[0] - intercept) / np.sqrt(
                m**2 + 1
            )
            if distance < closest_distance:
                closest_distance = distance
                closest_object = i
        # print(f"Closest Object: {closest_object}, Distance: {closest_distance}")
        return closest_object, closest_distance, visualize_img

    def draw_orthogonal_distance(self, m, intercept, centroid, img):
        # draw orthogonal line
        orthogonal_m = -1 / m
        orthogonal_intercept = centroid[1] - orthogonal_m * centroid[0]
        orthogonal_line_start = (0, int(orthogonal_intercept * img.shape[0]))
        orthogonal_line_end = (
            img.shape[1],
            int((orthogonal_m * 1 + orthogonal_intercept) * img.shape[0]),
        )
        cv2.line(img, orthogonal_line_start, orthogonal_line_end, (0, 0, 255), 2)
        return img


def main():
    rclpy.init()
    server = DetectPointingObjectServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
