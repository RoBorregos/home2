#!/usr/bin/env python3

"""
Test the /vision/chairs_to_remove service with an image from disk (no camera
needed). Publishes the image on the topics the pipeline consumes
(/vision/camera/image_oriented for hric_commands + YOLO, and the raw camera
topic for moondream), calls the service, prints the result and saves the
annotated frame next to the input image.

Needs object_detector_2d, moondream_node (+ gRPC server) and hric_commands
running. Run it WITHOUT the camera / image_orienter, so the disk image is the
only frame the nodes see.

Usage:
    ros2 run vision_general test_chairs_to_remove.py <image> [-o annotated.png]
"""

import argparse
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    CHAIR_REMOVAL_IMAGE_TOPIC,
    CHAIRS_TO_REMOVE_SERVICE,
    IMAGE_ORIENTED_TOPIC,
)
from frida_interfaces.srv import ChairsToRemove

WARMUP_SECONDS = 2.0  # let the nodes receive a few frames before the call
ANNOTATED_WAIT_SECONDS = 3.0  # hric_commands republishes the frame at 2 Hz


class TestChairsToRemove(Node):
    def __init__(self, frame):
        super().__init__("test_chairs_to_remove")
        self.bridge = CvBridge()
        self.image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.annotated = None

        self.oriented_pub = self.create_publisher(Image, IMAGE_ORIENTED_TOPIC, 10)
        self.camera_pub = self.create_publisher(Image, CAMERA_TOPIC, 10)
        self.create_timer(0.1, self.publish_frame)
        self.create_subscription(
            Image, CHAIR_REMOVAL_IMAGE_TOPIC, self.annotated_callback, 10
        )
        self.client = self.create_client(ChairsToRemove, CHAIRS_TO_REMOVE_SERVICE)

    def publish_frame(self):
        self.image_msg.header.stamp = self.get_clock().now().to_msg()
        self.oriented_pub.publish(self.image_msg)
        self.camera_pub.publish(self.image_msg)

    def annotated_callback(self, msg):
        self.annotated = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def spin_for(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def call_service(self, timeout):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"{CHAIRS_TO_REMOVE_SERVICE} not available (is hric_commands running?)"
            )
            return None
        self.spin_for(WARMUP_SECONDS)
        future = self.client.call_async(ChairsToRemove.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()


def main():
    parser = argparse.ArgumentParser(
        description="Test /vision/chairs_to_remove with an image from disk"
    )
    parser.add_argument("image", help="path to the input image")
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="where to save the annotated image (default: <image>_annotated.png)",
    )
    parser.add_argument(
        "--timeout", type=float, default=90.0, help="service timeout in seconds"
    )
    args, ros_args = parser.parse_known_args()

    frame = cv2.imread(args.image)
    if frame is None:
        sys.exit(f"Could not read image: {args.image}")

    rclpy.init(args=ros_args)
    node = TestChairsToRemove(frame)
    result = node.call_service(args.timeout)

    if result is None:
        node.get_logger().error("Service call failed or timed out")
    else:
        node.get_logger().info(
            f"success={result.success} table_found={result.table_found} "
            f"message='{result.message}'"
        )
        node.get_logger().info(
            f"{len(result.chairs)}/{result.total_chairs} chair(s) to remove"
        )
        for det in result.chairs:
            node.get_logger().info(
                f"  ({det.x1}, {det.y1}) -> ({det.x2}, {det.y2})  "
                f"conf={det.confidence:.2f}"
            )

        # The annotated frame only exists when chairs and table were detected
        if result.success and result.table_found and result.total_chairs > 0:
            node.spin_for(ANNOTATED_WAIT_SECONDS)
            if node.annotated is not None:
                output = args.output or (
                    args.image.rsplit(".", 1)[0] + "_annotated.png"
                )
                cv2.imwrite(output, node.annotated)
                node.get_logger().info(f"Annotated image saved to {output}")
            else:
                node.get_logger().warn("No annotated frame received")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
