#!/usr/bin/env python3

"""
Republishes the camera image rotated by an Int16 (0/90/180/270) so 2D
detectors consume an upright frame. Depth, camera_info and the TF tree
are left untouched; detectors that need 3D must use raw depth and pass
`rotation` to `point2d_to_ros_point_stamped`.
"""

import cv2
import rclpy
import rclpy.qos
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16

from frida_constants.vision_constants import (
    CAMERA_ROTATION_TOPIC,
    CAMERA_TOPIC,
    IMAGE_ORIENTED_TOPIC,
)

VALID_ROTATIONS = {0, 90, 180, 270}


class ImageOrienter(Node):
    def __init__(self):
        super().__init__("image_orienter")
        self.bridge = CvBridge()
        self.rotation = 0

        img_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, CAMERA_TOPIC, self._image_cb, img_qos)
        self.create_subscription(Int16, CAMERA_ROTATION_TOPIC, self._rotation_cb, 10)

        self.pub = self.create_publisher(Image, IMAGE_ORIENTED_TOPIC, img_qos)

        self.get_logger().info("Image orienter ready (rotation=0)")

    def _rotation_cb(self, msg: Int16):
        value = int(msg.data) % 360
        if value not in VALID_ROTATIONS:
            self.get_logger().warn(
                f"Ignoring unsupported rotation {msg.data}; expected one of {sorted(VALID_ROTATIONS)}"
            )
            return
        if value == self.rotation:
            return
        self.rotation = value
        self.get_logger().info(f"Camera rotation set to {self.rotation}")

    def _image_cb(self, msg: Image):
        if self.rotation == 0:
            self.pub.publish(msg)
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        if self.rotation == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotation == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotation == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        out.header = msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImageOrienter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
