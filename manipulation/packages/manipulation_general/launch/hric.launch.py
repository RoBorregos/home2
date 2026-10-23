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
            # perception_3d.launch.py
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
            Node(
                package="frida_motion_planning",
                executable="motion_planning_server.py",
            ),
            Node(
                package="manipulation_general",
                executable="manipulation_safeguard.py",
                output="screen",
                emulate_tty=True,
            ),
            # HRIC needs GoToHand, /manipulation/fixed_distance_move and the follow services.
            Node(
                package="pick_and_place",
                executable="manipulation_core.py",
                name="manipulation_core",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
