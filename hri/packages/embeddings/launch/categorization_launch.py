import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("embeddings"),
        "config",
        "categorization.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="embeddings",
                executable="categorization.py",
                name="embeddings",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
