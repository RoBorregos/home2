import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("object_detector_2d"), "config", "parameters_local.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="object_detector_2d",
                executable="object_detector_node.py",
                name="ObjectDetect2D",
                respawn=True,
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
        ]
    )
