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
                package="task_manager",
                executable="follow_face_node.py",
                name="follow_face_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
