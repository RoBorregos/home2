import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


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
                # parameters=[config],
            ),
            Node(
                package="vision_general",
                executable="gpsr_commands.py",
                name="gpsr_commands",
                output="screen",
                emulate_tty=True,
                # parameters=[config],
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(detector_launch_file)
            # ),
        ]
    )
