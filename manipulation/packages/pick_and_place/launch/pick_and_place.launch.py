#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pick_and_place",
                executable="manipulation_core.py",
                name="manipulation_core",
                output="screen",
                emulate_tty=True,
                parameters=[],
            ),
            Node(
                package="pick_and_place",
                executable="manipulation_server.py",
                name="manipulation_server",
                output="screen",
                emulate_tty=True,
                parameters=[],
            ),
            Node(
                package="pick_and_place",
                executable="pick_server.py",
                name="pick_server",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "ee_link_offset": -0.125,  # based on distance between end-effector link and contact point with objects e.g. where you grip
                    }
                ],
            ),
        ]
    )
