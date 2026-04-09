#!/usr/bin/env python3

"""
Centralized image rotator node.
Sits between the ZED camera and all vision consumers.
Subscribes to raw ZED RGB + depth topics, optionally rotates 180 degrees,
and republishes on intermediate topics consumed by all vision nodes.

Toggle rotation at runtime by publishing to /vision/flip_image (std_msgs/Bool).
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

from frida_constants.vision_constants import (
    CAMERA_INFO_TOPIC,
    CAMERA_TOPIC,
    DEPTH_IMAGE_TOPIC,
    FLIP_IMAGE_TOPIC,
    ZED_DEPTH_TOPIC,
    ZED_RGB_TOPIC,
    ZED_CAMERA_INFO_TOPIC,
)


class ImageRotatorNode(Node):
    def __init__(self):
        super().__init__("image_rotator")
        self.bridge = CvBridge()
        self.flip = False

        img_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        # Subscribers to raw ZED topics
        self.create_subscription(Image, ZED_RGB_TOPIC, self._rgb_callback, img_qos)
        self.create_subscription(Image, ZED_DEPTH_TOPIC, self._depth_callback, img_qos)
        self.create_subscription(
            CameraInfo, ZED_CAMERA_INFO_TOPIC, self._info_callback, img_qos
        )

        # Flip toggle
        self.create_subscription(Bool, FLIP_IMAGE_TOPIC, self._flip_callback, 10)

        # Publishers on intermediate topics
        self.rgb_pub = self.create_publisher(Image, CAMERA_TOPIC, img_qos)
        self.depth_pub = self.create_publisher(Image, DEPTH_IMAGE_TOPIC, img_qos)
        self.info_pub = self.create_publisher(CameraInfo, CAMERA_INFO_TOPIC, img_qos)

        self.get_logger().info(
            f"Image rotator ready (flip={self.flip}). "
            f"Subscribe to {FLIP_IMAGE_TOPIC} to toggle."
        )

    def _flip_callback(self, msg: Bool):
        if msg.data != self.flip:
            self.flip = msg.data
            self.get_logger().info(f"Flip image set to: {self.flip}")

    def _rgb_callback(self, msg: Image):
        if self.flip:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            out.header = msg.header
            self.rgb_pub.publish(out)
        else:
            self.rgb_pub.publish(msg)

    def _depth_callback(self, msg: Image):
        if self.flip:
            frame = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            out = self.bridge.cv2_to_imgmsg(frame, "32FC1")
            out.header = msg.header
            self.depth_pub.publish(out)
        else:
            self.depth_pub.publish(msg)

    def _info_callback(self, msg: CameraInfo):
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRotatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
