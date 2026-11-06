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
    semantic_map_config = os.path.join(
        get_package_share_directory("vision_general"), "config", "semantic_map.yaml"
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch_file)
            ),
            Node(
                package="vision_general",
                executable="finals_commands.py",
                name="finals_commands",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="vision_general",
                executable="semantic_map_node.py",
                name="semantic_map_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[semantic_map_config],
            ),
        ]
    )
