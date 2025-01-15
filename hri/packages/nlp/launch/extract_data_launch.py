from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="nlp",
                executable="extract_data.py",
                name="extract_data",
                output="screen",
                emulate_tty=True,
                parameters=[],
            ),
        ]
    )
