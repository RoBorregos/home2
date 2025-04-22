import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    detector_launch_file = os.path.join(
        get_package_share_directory("object_detector_2d"),
        "launch",
        "zero_shot_object_detector_node.launch.py",
    )
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch_file)
            ),
        ]
    )
