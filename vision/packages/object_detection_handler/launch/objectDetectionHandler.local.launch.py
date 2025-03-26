#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("object_detection_handler"),
        "config",
        "parameters_local.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="object_detection_handler",
                executable="object_detection_handler",
                name="object_detection_handler",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
        ]
    )
