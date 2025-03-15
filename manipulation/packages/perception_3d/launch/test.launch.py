#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception_3d",
                executable="pick_primitives",
                name="pick_primitives",
                output="screen",
                emulate_tty=True,
                parameters=[],
            ),
            Node(
                package="perception_3d",
                executable="plane_service",
                name="plane_service",
                output="screen",
                emulate_tty=True,
                parameters=[],
            ),
            Node(
                package="perception_3d",
                executable="test_only_orchestrator",
                name="test_only_orchestrator",
                output="screen",
                emulate_tty=True,
                parameters=[],
            ),
        ]
    )
