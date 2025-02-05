from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="embeddings",
                executable="item_categorization.py",  # Your ROS 2 node executable
                name="embeddings",
                output="screen",
                parameters=[
                    {"collections_built": 0}
                ],  # Default value (0 means not built)
            ),
        ]
    )
