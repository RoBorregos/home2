#!/usr/bin/env python3

"""
Detect a circular hole in a wall (e.g. a washing-machine porthole or
laundry-chute opening) and return its centroid as a 3D point.

Used by the doing-laundry task. The detection is purely OpenCV-based
(grayscale + Gaussian blur + HoughCircles), so no model files are needed.
"""

import cv2
import numpy as np
import rclpy
import rclpy.qos
import rclpy.callback_groups
import rclpy.executors
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from frida_constants.vision_constants import (
    CAMERA_FRAME,
    CAMERA_INFO_TOPIC,
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    WALL_HOLE_DEBUG_IMAGE_TOPIC,
    WALL_HOLE_DETECTION_TOPIC,
)
from frida_interfaces.msg import ObjectDetection, ObjectDetectionArray
from frida_interfaces.srv import DetectCircleHole
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_general.utils.calculations import point2d_to_ros_point_stamped

# Hough parameters tuned for a washer-porthole-sized circle at typical
# robot working distance. Override per-deployment if needed.
HOUGH_DP = 1.2
HOUGH_MIN_DIST = 80
HOUGH_PARAM1 = 100
HOUGH_PARAM2 = 30
HOUGH_MIN_RADIUS = 30
HOUGH_MAX_RADIUS = 300


class WallHoleDetectionNode(Node):
    def __init__(self):
        super().__init__("wall_hole_detection")
        self.bridge = CvBridge()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.image = None
        self.image_info = None
        self.depth_image = None

        self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, qos)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.image_info_callback, qos
        )
        self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self.image_depth_callback, qos
        )

        self.service = self.create_service(
            DetectCircleHole,
            WALL_HOLE_DETECTION_TOPIC,
            self.detect_circle_hole_callback,
            callback_group=self.callback_group,
        )

        self.debug_publisher = self.create_publisher(
            Image, WALL_HOLE_DEBUG_IMAGE_TOPIC, 10
        )

        self.get_logger().info("Wall hole detection node initialized")

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def image_info_callback(self, msg):
        self.image_info = msg

    def image_depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def find_circle_centroid(self, image):
        """Run HoughCircles on a BGR image and return the strongest circle.

        Returns (cx, cy, radius) or None if no circle was detected.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=HOUGH_DP,
            minDist=HOUGH_MIN_DIST,
            param1=HOUGH_PARAM1,
            param2=HOUGH_PARAM2,
            minRadius=HOUGH_MIN_RADIUS,
            maxRadius=HOUGH_MAX_RADIUS,
        )

        if circles is None:
            return None

        circles = np.round(circles[0, :]).astype(int)
        # HoughCircles orders results by accumulator strength (best first).
        cx, cy, r = circles[0]
        return int(cx), int(cy), int(r)

    def publish_debug_image(self, circle):
        if self.image is None:
            return
        debug = self.image.copy()
        if circle is not None:
            cx, cy, r = circle
            cv2.circle(debug, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(debug, (cx, cy), 3, (0, 0, 255), -1)
        self.debug_publisher.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

    def detect_circle_hole_callback(self, request, response):
        self.get_logger().info("Received wall hole detection request")
        response.detection_array = ObjectDetectionArray()
        response.detection_array.detections = []
        response.success = False

        if self.image is None or self.depth_image is None or self.image_info is None:
            self.get_logger().warn("Missing image, depth, or camera info")
            return response

        circle = self.find_circle_centroid(self.image)
        if circle is None:
            self.get_logger().warn("No circular hole detected")
            self.publish_debug_image(None)
            return response

        cx, cy, r = circle
        point_3d = point2d_to_ros_point_stamped(
            self.image_info,
            self.depth_image,
            (cx, cy),
            CAMERA_FRAME,
            Time(sec=0, nanosec=0),
        )

        detection = ObjectDetection(
            label=0,
            label_text="wall_hole",
            score=1.0,
            xmin=float(cx - r),
            ymin=float(cy - r),
            xmax=float(cx + r),
            ymax=float(cy + r),
            point3d=point_3d,
        )
        response.detection_array.detections = [detection]
        response.success = True

        self.get_logger().info(
            f"Wall hole at pixel=({cx},{cy}) r={r} -> 3D=("
            f"{point_3d.point.x:.3f},{point_3d.point.y:.3f},{point_3d.point.z:.3f})"
        )
        self.publish_debug_image(circle)
        return response


if __name__ == "__main__":
    rclpy.init()
    node = WallHoleDetectionNode()
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
