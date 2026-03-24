from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_general",
                executable="face_recognition_node.py",
                name="face_recognition",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="vision_general",
                executable="hric_commands.py",
                name="hric_commands",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="vision_general",
                executable="yolo_node.py",
                name="yolo_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
