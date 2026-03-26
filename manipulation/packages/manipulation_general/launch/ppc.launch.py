#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            # MoveIt config
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("arm_pkg"),
                            "launch",
                            "frida_moveit_config.launch.py",
                        ]
                    )
                ),
            ),
            # Pick and place stack (includes GPD, manipulation_core, pick/place/pour servers,
            # perception_3d, heatmap, motion planning, fix_position_to_plane, flat_grasp_estimator)
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
            ),
        ]
    )