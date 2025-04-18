from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_general",
                executable="storing_groceries_commands.py",
                name="storing_groceries_commands",
                output="screen",
                emulate_tty=True,
                # parameters=[config],
            )
        ]
    )
