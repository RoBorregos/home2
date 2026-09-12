import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("object_detector_2d"),
        "config",
        "parameters_zero_shot.yaml",
    )

    # Native ROS 2 Remapping:
    # Instead of an expensive Python relay, we tell the node to publish
    # directly to /vision/detections. This has ZERO overhead and latency.
    remappings = [
        ("/vision/zero_shot_detections", "/vision/detections"),
    ]

    return LaunchDescription(
        [
            Node(
                package="vision_general",
                executable="image_orienter.py",
                name="image_orienter",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="object_detector_2d",
                executable="zero_shot_object_detector_node.py",
                name="ZeroShotDetect2D",
                respawn=True,
                output="screen",
                emulate_tty=True,
                parameters=[config],
                remappings=remappings,
            ),
        ]
    )