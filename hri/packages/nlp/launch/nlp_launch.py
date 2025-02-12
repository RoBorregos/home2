import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    command_interpreter_config = os.path.join(
        get_package_share_directory("nlp"), "config", "command_interpreter.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="nlp",
                executable="command_interpreter.py",
                name="command_interpreter",
                output="screen",
                emulate_tty=True,
                parameters=[command_interpreter_config],
            ),
        ]
    )
