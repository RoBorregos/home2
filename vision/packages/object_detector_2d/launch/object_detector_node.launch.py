import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_gallery = LaunchConfiguration("enable_gallery")
    config = os.path.join(
        get_package_share_directory("object_detector_2d"), "config", "parameters.yaml"
    )
    config_gallery = os.path.join(
        get_package_share_directory("object_detector_2d"),
        "config",
        "parameters_gallery.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_gallery",
                default_value="false",
                description="Start the embedding gallery detector",
            ),
            Node(
                package="vision_general",
                executable="image_orienter.py",
                name="image_orienter",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="object_detector_2d",
                executable="object_detector_node.py",
                name="ObjectDetect2D",
                respawn=True,
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
            Node(
                package="object_detector_2d",
                executable="gallery_object_detector_node.py",
                name="GalleryDetect2D",
                respawn=True,
                output="screen",
                emulate_tty=True,
                parameters=[config_gallery],
                condition=IfCondition(enable_gallery),
            ),
        ]
    )
