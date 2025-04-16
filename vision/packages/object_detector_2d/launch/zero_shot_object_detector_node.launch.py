import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("object_detector_2d"), "config", "parameters.yaml"
    )
    handler_launch_file = os.path.join(
        get_package_share_directory("object_detection_handler"),
        "launch",
        "objectDetectionHandler.launch.py",
    )
    return LaunchDescription(
        [
            Node(
                package="object_detector_2d",
                executable="zero_shot_object_detector_node.py",
                name="ObjectDetect2D",
                respawn=True,
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(handler_launch_file)
            ),
        ]
    )
