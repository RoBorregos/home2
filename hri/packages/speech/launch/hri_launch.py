#!/usr/bin/env python3

import logging
import os

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


class _DropProcessStarted(logging.Filter):
    """Hide launch's per-process "process started" lines.

    They go through the same per-process logger as the node's own screen output,
    so raising that logger's level would silence the node too; filter by text instead.
    """

    def filter(self, record):
        return not record.getMessage().startswith("process started with pid")


def generate_launch_description():
    if os.environ.get("HRI_LOG_LEVEL", "info").lower() != "debug":
        launch.logging.launch_config.get_screen_handler().addFilter(
            _DropProcessStarted()
        )

    speech_launch_path = os.path.join(
        get_package_share_directory("speech"), "launch", "devices_launch.py"
    )
    nlp_launch_path = os.path.join(
        get_package_share_directory("nlp"), "launch", "nlp_launch.py"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(speech_launch_path)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(nlp_launch_path)),
        ]
    )
