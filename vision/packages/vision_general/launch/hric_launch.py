import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("object_detector_2d"), "config", "parameters.yaml"
    )
    return LaunchDescription(
        [
            # start_active:=true runs face_recognition without the task manager.
            DeclareLaunchArgument("start_active", default_value="false"),
            Node(
                package="vision_general",
                executable="face_recognition_node.py",
                name="face_recognition",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "start_active": ParameterValue(
                            LaunchConfiguration("start_active"), value_type=bool
                        )
                    }
                ],
            ),
            Node(
                package="vision_general",
                executable="hric_commands.py",
                name="hric_commands",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="moondream_run",
                executable="moondream_node.py",
                name="moondream_node",
                output="screen",
                emulate_tty=True,
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
                parameters=[config, {"models": [""]}],
            ),
            Node(
                package="vision_general",
                executable="tracker_node.py",
                name="tracker_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="vision_general",
                executable="talking_detection_node.py",
                name="talking_detection",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
