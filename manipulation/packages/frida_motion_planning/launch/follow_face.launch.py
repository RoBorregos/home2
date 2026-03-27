from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    follow_face_node = Node(
        package="frida_motion_planning",
        executable="follow_face_node.py",
        name="follow_face_node",
        output="screen",
    )

    return LaunchDescription([follow_face_node])
