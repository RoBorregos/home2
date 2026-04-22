import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    yolo_model_path = LaunchConfiguration(
        "yolo_model_path", default="tmr_30classes_v2.pt"
    )
    config = os.path.join(
        get_package_share_directory("object_detector_2d"), "config", "parameters.yaml"
    )
    return LaunchDescription(
        [
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
                        "YOLO_MODEL_PATH": yolo_model_path,
                    },
                ],
            ),
        ]
    )
