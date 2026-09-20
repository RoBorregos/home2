#!/usr/bin/env python3
"""Real manipulation stack (MoveIt + pick_and_place) on sim time."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("show_rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("frida_gz_sim"),
                            "launch",
                            "sim_moveit.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "show_rviz": LaunchConfiguration("show_rviz")
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("pick_and_place"),
                            "launch",
                            "pick_and_place.launch.py",
                        ]
                    )
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
        ]
    )
