from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("nlp"), "config", "stop_listener.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="nlp",
                executable="stop_listener.py",
                name="stop_listener",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
        ]
    )
