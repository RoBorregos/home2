#!/usr/bin/env python3
"""RViz view of the sim: robot, camera cloud, detections and the MoveIt planning scene."""

import os

from ament_index_python.packages import get_package_share_directory
from frida_gz_sim.moveit_config import build_moveit_config
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    moveit_config = build_moveit_config(context)
    rviz_config = os.path.join(
        get_package_share_directory("frida_gz_sim"), "config", "sim.rviz"
    )
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"use_sim_time": True},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
