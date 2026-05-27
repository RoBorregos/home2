#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "grasp_backend",
                default_value="contact_graspnet",
                description="Grasp backend: 'contact_graspnet' (default) or 'gpd'. "
                "Override on launch: ros2 launch ... ppc.launch.py grasp_backend:=gpd",
            ),
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
            # Pick and place stack (grasp backend selectable via grasp_backend arg).
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
                launch_arguments={
                    "grasp_backend": LaunchConfiguration("grasp_backend"),
                }.items(),
            ),
        ]
    )
