#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from frida_constants import ModuleNames, parse_ros_config


def generate_launch_description():
    command_interpreter_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("nlp"), "config", "command_interpreter.yaml"
        ),
        [ModuleNames.HRI.value],
    )["command_interpreter"]["ros__parameters"]

    extract_data_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("nlp"),
            "config",
            "extract_data.yaml",
        ),
        [ModuleNames.HRI.value],
    )["extract_data"]["ros__parameters"]

    llm_utils_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("nlp"),
            "config",
            "llm_utils.yaml",
        ),
        [ModuleNames.HRI.value],
    )["llm_utils"]["ros__parameters"]

    return LaunchDescription(
        [
            Node(
                package="nlp",
                executable="command_interpreter.py",
                name="command_interpreter",
                output="screen",
                emulate_tty=True,
                parameters=[command_interpreter_config],
            ),
            Node(
                package="nlp",
                executable="extract_data.py",
                name="extract_data",
                output="screen",
                emulate_tty=True,
                parameters=[extract_data_config],
            ),
            Node(
                package="nlp",
                executable="llm_utils.py",
                name="llm_utils",
                output="screen",
                emulate_tty=True,
                parameters=[llm_utils_config],
            ),
        ]
    )
