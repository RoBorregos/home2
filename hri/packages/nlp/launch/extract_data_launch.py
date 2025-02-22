from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("nlp"), "config", "extract_data.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="nlp",
                executable="extract_data.py",
                name="extract_data",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
        ]
    )
