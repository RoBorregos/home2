"""Vision launch for the Doing Laundry task.

The laundry task manager's only vision dependency is detect_objects
(DetectionHandler / object_detector_node), so this launch brings up just the
detector include (which also starts image_orienter). No moondream_node here:
--dlc does not enable the moondream-server compose profile, and the task makes
no moondream calls.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch_file)
            ),
        ]
    )
