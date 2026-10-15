#!/usr/bin/env python3
"""Real 2D object detector pipeline on the simulated ZED stream."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    detector_params = os.path.join(
        get_package_share_directory("object_detector_2d"), "config", "parameters.yaml"
    )
    sim_params = os.path.join(
        get_package_share_directory("frida_gz_sim"), "config", "detector_sim.yaml"
    )
    return LaunchDescription(
        [
            # image_orienter.py is not executable in the repo, so run it through python3
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(
                        get_package_prefix("vision_general"),
                        "lib",
                        "vision_general",
                        "image_orienter.py",
                    ),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
                output="screen",
            ),
            Node(
                package="frida_gz_sim",
                executable="sim_object_detector.py",
                name="ObjectDetect2D",
                respawn=True,
                output="screen",
                emulate_tty=True,
                parameters=[detector_params, sim_params],
            ),
        ]
    )
