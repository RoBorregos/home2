import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from frida_constants import ModuleNames, parse_ros_config


def generate_launch_description():
    item_categorization_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("embeddings"),
            "config",
            "item_categorization.yaml",
        ),
        [ModuleNames.HRI.value],
    )["item_categorization"]["ros__parameters"]

    return LaunchDescription(
        [
            Node(
                package="embeddings",
                executable="item_categorization.py",  # Your ROS 2 node executable
                name="embeddings",
                output="screen",
                parameters=[
                    item_categorization_config
                ],  # Default value (0 means not built)
            ),
        ]
    )
