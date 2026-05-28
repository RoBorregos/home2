#!/usr/bin/env python3
"""Launch the object-aware pointcloud filter node.

Sits between `downsample_pc` (publishes /point_cloud) and move_group's
octomap monitor (which subscribes to whatever `point_cloud_topic` is set
in sensors_3d.yaml). When perception publishes a target bbox on
/manipulation/target_object_bbox, the filter drops the matching points
from the cloud before they reach the octomap. When no bbox is active
(or it's older than `bbox_ttl_sec`), pass-through.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic", default="/point_cloud")
    output_topic = LaunchConfiguration(
        "output_topic", default="/point_cloud_object_filtered"
    )
    bbox_topic = LaunchConfiguration(
        "bbox_topic", default="/manipulation/target_object_bbox"
    )
    bbox_ttl_sec = LaunchConfiguration("bbox_ttl_sec", default="30.0")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/point_cloud"),
            DeclareLaunchArgument(
                "output_topic", default_value="/point_cloud_object_filtered"
            ),
            DeclareLaunchArgument(
                "bbox_topic", default_value="/manipulation/target_object_bbox"
            ),
            DeclareLaunchArgument("bbox_ttl_sec", default_value="30.0"),
            Node(
                package="perception_3d",
                executable="object_aware_pointcloud_filter.py",
                name="object_aware_pointcloud_filter",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                        "bbox_topic": bbox_topic,
                        "bbox_ttl_sec": bbox_ttl_sec,
                    }
                ],
            ),
        ]
    )
