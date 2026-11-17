#!/usr/bin/env python3
"""Feeds MoveIt's octomap a cloud with the graspable objects removed.

Gazebo's depth camera is noise-free and dense, so every object on the table is
reconstructed as a solid block of occupied voxels. MoveIt then reports the gripper
in collision with the very object it is reaching for and every GPD candidate comes
back "grasp pose unreachable" (confirmed with /compute_ik + /check_state_validity).
The real ZED cloud is sparse enough that the same grasps clear.

Dropping everything above the table surface keeps the table, the floor and the
robot's surroundings in the octomap - so the arm still avoids them - while leaving
the objects to the planning-scene collision objects the pick pipeline adds.
Points are filtered in base_link but republished in the sensor frame, because the
octomap updater needs the sensor origin.
"""

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class OctomapCloudFilter(Node):
    def __init__(self):
        super().__init__("octomap_cloud_filter")
        input_topic = self.declare_parameter("input_topic", "/point_cloud").value
        output_topic = self.declare_parameter(
            "output_topic", "/sim/octomap_cloud"
        ).value
        self.reference_frame = self.declare_parameter(
            "reference_frame", "base_link"
        ).value
        # Just above the dining table top, so the table stays and the objects go
        self.max_z = self.declare_parameter("max_z", 0.76).value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, input_topic, self._filter, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Octomap cloud filter ready: {input_topic} -> {output_topic} "
            f"(dropping {self.reference_frame} z > {self.max_z})"
        )

    def _filter(self, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame, msg.header.frame_id, rclpy.time.Time()
            ).transform
        except tf2_ros.TransformException:
            return
        points = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if not len(points):
            return
        rotation = Rotation.from_quat(
            [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ]
        ).as_matrix()
        height = points @ rotation[2] + transform.translation.z
        self.pub.publish(
            point_cloud2.create_cloud_xyz32(msg.header, points[height <= self.max_z])
        )


def main(args=None):
    rclpy.init(args=args)
    node = OctomapCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
