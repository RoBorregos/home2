#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("object_detection_handler"),
        "config",
        "parameters.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (true) for the MuJoCo sim, wall time (false) for real robot.",
            ),
            Node(
                package="object_detection_handler",
                executable="object_detection_handler",
                name="object_detection_handler",
                output="screen",
                emulate_tty=True,
                parameters=[config, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
