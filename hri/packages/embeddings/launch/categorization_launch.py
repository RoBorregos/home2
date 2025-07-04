from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="embeddings",
                executable="knowledge_base.py",
                name="knowledge_base",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
