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
            DeclareLaunchArgument(
                "pose_service",
                default_value="false",
                description="Also load yolo11m-pose and serve DetectPose",
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
                parameters=[
                    config,
                    {
                        "POSE_SERVICE": ParameterValue(
                            LaunchConfiguration("pose_service"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
