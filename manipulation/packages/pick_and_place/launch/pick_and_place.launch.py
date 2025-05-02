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
            # gpd
            Node(
                package="arm_pkg",
                executable="gpd_service",
                name="gpd_service",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[],
            ),
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
                        # based on distance between end-effector link and contact point with objects e.g. where you grip
                        "ee_link_offset": -0.08,
                    }
                ],
            ),
            Node(
                package="pick_and_place",
                executable="place_server.py",
                name="place_server",
                output="screen",
                emulate_tty=True,
            ),
            # perception_3d.launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perception_3d"),
                            "launch",
                            "perception_3d.launch.py",
                        ]
                    )
                ),
            ),
            Node(
                package="place",
                executable="heatmapPlace_Server.py",
            ),
            Node(
                package="frida_motion_planning",
                executable="motion_planning_server.py",
            ),
            Node(
                package="frida_motion_planning",
                executable="fix_position_to_plane.py",
                name="fix_position_to_plane",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
