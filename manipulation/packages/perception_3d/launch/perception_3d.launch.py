#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    point_cloud_topic = LaunchConfiguration("point_cloud_topic", default="/point_cloud")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "point_cloud_topic",
                default_value="/point_cloud",
                description="Point cloud topic consumed by the cluster extraction.",            ),
            Node(
                package="perception_3d",
                executable="pick_primitives",
                name="pick_primitives",
                output="screen",
                emulate_tty=True,
                respawn=True,
            ),
            Node(
                package="perception_3d",
                executable="plane_service",
                name="plane_service",
                output="screen",s
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {"point_cloud_topic": point_cloud_topic},
                ],
            ),
            Node(
                package="perception_3d",
                executable="test_only_orchestrator",
                name="test_only_orchestrator",
                output="screen",
                respawn=True,
                emulate_tty=True,
                parameters=[
                    {"testing": False, "point_cloud_topic": point_cloud_topic},
                ],
            ),
        ]
    )