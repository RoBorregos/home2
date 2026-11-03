#!/usr/bin/env python3
"""Republishes the gz RGBD cloud on the ZED topic with the frame its points are expressed in."""

import time

import rclpy
from frida_constants.manipulation_constants import ZED_POINT_CLOUD_TOPIC
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudFrameFix(Node):
    def __init__(self):
        super().__init__("cloud_frame_fix")
        # gz rgbd points use the camera link convention (x forward), not the optical one
        self.frame_id = self.declare_parameter(
            "frame_id", "zed_left_camera_frame"
        ).value
        input_topic = self.declare_parameter(
            "input_topic", "/zed_rgbd/points_raw"
        ).value
        output_topic = self.declare_parameter(
            "output_topic", ZED_POINT_CLOUD_TOPIC
        ).value
        # Full-resolution clouds are several MB each; throttling keeps DDS from starving /tf
        self.min_period = 1.0 / self.declare_parameter("max_rate", 2.0).value
        self.last_pub = 0.0
        self.pub = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, input_topic, self._cb, qos_profile_sensor_data
        )
        self.get_logger().info(f"{input_topic} -> {output_topic} [{self.frame_id}]")

    def _cb(self, msg: PointCloud2):
        now = time.monotonic()
        if now - self.last_pub < self.min_period:
            return
        self.last_pub = now
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudFrameFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
