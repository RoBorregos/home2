from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_general",
                executable="tracker_node.py",
                name="tracker_node",
                output="screen",
                emulate_tty=True,
                # parameters=[config],
            ),
            Node(
                package="vision_general",
                executable="receptionist_commands.py",
                name="receptionist_commands",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="vision_general",
                executable="pointing_detection.py",
                name="detect_pointing_object_server",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
