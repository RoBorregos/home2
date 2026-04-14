from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    detector_launch_file = os.path.join(
        get_package_share_directory("object_detector_2d"),
        "launch",
        "object_detector_combined.launch.py",
    )
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
                executable="yolo_node.py",
                name="yolo_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="vision_general",
                executable="restaurant_commands.py",
                name="restaurant_commands",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="vision_general",
                executable="customer_node.py",
                name="customer_node",
                output="screen",
                emulate_tty=True,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch_file)
            ),
            Node(
                package="moondream_run",
                executable="moondream_node.py",
                name="moondream_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
