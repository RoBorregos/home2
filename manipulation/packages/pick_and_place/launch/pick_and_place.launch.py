#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            # gpd
            Node(
                package="arm_pkg",
                executable="gpd_service",
                name="gpd_service",
                # GPD's vendored C++ dumps every grasp to stdout with printf and no
                # flag silences it; ROS logs go to stderr, so only stdout is hidden.
                output={"stdout": "log", "stderr": "screen"},
                emulate_tty=True,
                respawn=True,
            ),
            Node(
                package="pick_and_place",
                executable="manipulation_core.py",
                name="manipulation_core",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        # based on distance between end-effector link and contact point with objects
                        "ee_link_offset": -0.09,
                    },
                    # Per-strategy tuning lives in pick_and_place/config/pick_profiles.yaml.
                ],
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
                launch_arguments={
                    "point_cloud_topic": LaunchConfiguration(
                        "point_cloud_topic", default="/point_cloud"
                    ),
                }.items(),
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
                package="manipulation_general",
                executable="manipulation_safeguard.py",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="pick_and_place",
                executable="fix_position_to_plane.py",
                name="fix_position_to_plane",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="perception_3d",
                executable="flat_grasp_estimator.py",
                name="flat_grasp_estimator",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )