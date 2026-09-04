#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    speech_launch_path = os.path.join(
        get_package_share_directory("speech"), "launch", "devices_launch.py"
    )
    nlp_launch_path = os.path.join(
        get_package_share_directory("nlp"), "launch", "nlp_launch.py"
    )
    display_launch_path = os.path.join(
        get_package_share_directory("display"), "launch", "display_launch.py"
    )
    display_launch_backup_path = os.path.join(
        get_package_share_directory("display"), "launch", "display_launch_backup.py"
    )

    display_task_arg = DeclareLaunchArgument(
        "display_task",
        default_value="default",
        description="View for the PyQt display UI (task arg of display_launch.py)",
    )
    display_backup_arg = DeclareLaunchArgument(
        "display_backup",
        default_value="false",
        description=(
            "Use the legacy Next.js display (display_launch_backup.py) instead "
            "of the default PyQt UI"
        ),
    )

    return LaunchDescription(
        [
            display_task_arg,
            display_backup_arg,
            IncludeLaunchDescription(PythonLaunchDescriptionSource(speech_launch_path)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(nlp_launch_path)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(display_launch_path),
                launch_arguments={"task": LaunchConfiguration("display_task")}.items(),
                condition=UnlessCondition(LaunchConfiguration("display_backup")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(display_launch_backup_path),
                condition=IfCondition(LaunchConfiguration("display_backup")),
            ),
        ]
    )
