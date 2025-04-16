#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # gpd
            # Node(
            #     package="arm_pkg",
            #     executable="gpd_service",
            #     name="gpd_service",
            #     output="screen",
            #     emulate_tty=True,
            #     parameters=[],
            # ),
            Node(
                package="pick_and_place",
                executable="manipulation_core.py",
                name="manipulation_core",
                output="screen",
                emulate_tty=True,
                parameters=[],
            ),
            # Node(
            #     package="pick_and_place",
            #     executable="manipulation_client.py",
            #     name="manipulation_client",
            #     output="screen",
            #     emulate_tty=True,
            #     parameters=[],
            # ),
            Node(
                package="pick_and_place",
                executable="pick_server.py",
                name="pick_server",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "ee_link_offset": -0.12,  # based on distance between end-effector link and contact point with objects e.g. where you grip
                    }
                ],
            ),
        ]
    )
