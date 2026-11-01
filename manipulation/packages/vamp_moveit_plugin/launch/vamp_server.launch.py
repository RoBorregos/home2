"""Starts the VAMP planning server that backs the "vamp" MoveIt pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("vamp_moveit_plugin"), "config", "vamp_server.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="vamp_moveit_plugin",
                executable="vamp_server.py",
                name="vamp_server",
                parameters=[params],
                output="screen",
                respawn=True,
            )
        ]
    )
