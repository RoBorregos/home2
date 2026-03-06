#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from frida_interfaces.vision.srv import DetectHand
from vision_general.utils.calculations import get_depth, deproject_pixel_to_point
from frida_constants.frida_constants.vision_constants import (
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_FRAME,
    DETECT_HAND_SERVICE,
)
import cv2
import mediapipe as mp
import threading


class DetectHandNode(Node):
    def __init__(self):
        super().__init__("detect_hand_node")
        self.get_logger().info("Initializing Detect Hand Node")

        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.image = None
        self.depth_image = None
        self.camera_info = None
        self.hand_point = None
        self.run_thread = None

        qos = rclpy.qos.QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.image_sub = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos
        )
        self.depth_sub = self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.depth_callback, qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, qos
        )

        self.service = self.create_service(
            DetectHand, DETECT_HAND_SERVICE, self.service_callback
        )

        self.get_logger().info("Detect Hand Node Initialized")

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.run_thread is None or not self.run_thread.is_alive():
            self.run_thread = threading.Thread(
                target=self.run_inference, daemon=True
            )
            self.run_thread.start()

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def run_inference(self):
        if self.image is None:
            return

        image_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = self.hands.process(image_rgb)

        if not results.multi_hand_landmarks:
            self.hand_point = None
            return

        hand_landmarks = results.multi_hand_landmarks[0]
        lm = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]

        h, w, _ = self.image.shape
        px, py = int(lm.x * w), int(lm.y * h)

        if self.depth_image is not None and self.camera_info is not None:
            depth = get_depth(self.depth_image, (px, py))
            point3d = deproject_pixel_to_point(self.camera_info, (px, py), depth)

            stamped = PointStamped()
            stamped.header.frame_id = CAMERA_FRAME
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.point.x = float(point3d[0])
            stamped.point.y = float(point3d[1])
            stamped.point.z = float(point3d[2])
            self.hand_point = stamped
        else:
            self.hand_point = None

    def service_callback(self, request, response):
        if self.hand_point is not None:
            response.point = self.hand_point
            response.success = True
            self.get_logger().info(
                f"Hand detected at ({self.hand_point.point.x:.3f}, "
                f"{self.hand_point.point.y:.3f}, {self.hand_point.point.z:.3f})"
            )
        else:
            response.success = False
            self.get_logger().info("No hand detected")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DetectHandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
