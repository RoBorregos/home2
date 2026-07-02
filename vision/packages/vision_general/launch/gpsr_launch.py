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
        "object_detector_node.launch.py",
    )
    return LaunchDescription(
        [
            Node(
                package="vision_general",
                executable="hric_commands.py",
                name="hric_commands",
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
            # image_orienter (feeds IMAGE_ORIENTED_TOPIC for gpsr_commands,
            # hric_commands, face_recognition and the display's default video
            # feed) is started by the included object_detector_node.launch.py —
            # do NOT add it here again or two instances will run.
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
                executable="trash_detection_node.py",
                name="trash_detection_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="vision_general",
                executable="tracker_node.py",
                name="tracker_node",
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
