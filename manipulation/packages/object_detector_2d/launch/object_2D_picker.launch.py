import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("object_detector_2d"), "config", "parameters.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="object_detector_2d",
                executable="Detection2D.py",
                name="Detection2D",
                respawn=True,
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
            Node(
                package="object_detector_2d",
                exec="DetectionPicker.py",
                name="DetectionPicker",
                respawn="true",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
        ]
    )
