#!/usr/bin/env python3
"""Republishes the gz RGBD cloud on the ZED topic with the frame its points are expressed in."""

import os
import time

import numpy as np
import rclpy
import tf2_ros
from frida_constants.manipulation_constants import ZED_POINT_CLOUD_TOPIC
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2

# The A2 head camera sees its own shoulders and arms: drop points above shoulder height near the body (base_link)
SELF_FILTER_MIN_Z = 0.97
SELF_FILTER_MAX_X = 0.3


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
        self.self_filter = os.environ.get("FRIDA_ARM") == "a2"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
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
        if self.self_filter and not self._drop_robot_points(msg):
            return
        self.pub.publish(msg)

    def _drop_robot_points(self, msg: PointCloud2) -> bool:
        """NaN out the robot's own points in place, keeping the cloud organized."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id, rclpy.time.Time()
            ).transform
        except tf2_ros.TransformException:
            return False
        q, t = transform.rotation, transform.translation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        data = (
            np.frombuffer(msg.data, dtype=np.uint8).copy().reshape(-1, msg.point_step)
        )
        offsets = {f.name: f.offset for f in msg.fields}
        xyz = np.stack(
            [
                data[:, offsets[c] : offsets[c] + 4].copy().view(np.float32)[:, 0]
                for c in "xyz"
            ],
            axis=1,
        )
        points = xyz @ rotation.T + np.array([t.x, t.y, t.z])
        robot = (points[:, 2] > SELF_FILTER_MIN_Z) & (points[:, 0] < SELF_FILTER_MAX_X)
        nan = np.full(1, np.nan, dtype=np.float32).view(np.uint8)
        for c in "xyz":
            data[robot, offsets[c] : offsets[c] + 4] = nan
        msg.data = data.tobytes()
        return True


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
