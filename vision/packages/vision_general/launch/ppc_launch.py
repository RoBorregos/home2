from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    detector_launch_file = os.path.join(
        get_package_share_directory("object_detector_2d"),
        "launch",
        "object_detector_node.launch.py",
    )
    return LaunchDescription(
        [
            # trash_detection_node removed: no task_manager client for its TRASHCAN
            # service exists (detect_trash() uses moondream instead).
            # hric_commands serves CHAIRS_TO_REMOVE_SERVICE, which the
            # pick-and-place TM calls before docking at the dining table.
            Node(
                package="vision_general",
                executable="hric_commands.py",
                name="hric_commands",
                output="screen",
                emulate_tty=True,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch_file)
            ),
            Node(
                package="moondream_run",
                executable="moondream_node.py",
                name="moondream_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
