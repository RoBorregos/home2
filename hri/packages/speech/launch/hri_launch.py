#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Find the package containing the included launch files
    speech_launch_path = os.path.join(
        get_package_share_directory("speech"), "launch", "devices_launch.py"
    )
    nlp_launch_path = os.path.join(
        get_package_share_directory("nlp"), "launch", "nlp_launch.py"
    )
    embeddings_launch_path = os.path.join(
        get_package_share_directory("embeddings"), "launch", "chroma_launch.py"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(speech_launch_path)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(nlp_launch_path)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(embeddings_launch_path)
            ),
        ]
    )
