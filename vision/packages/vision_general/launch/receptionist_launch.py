from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


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
                executable="receptionist_commands.py",
                name="receptionist_commands",
                output="screen",
                emulate_tty=True,
                # parameters=[config],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch_file),
                launch_arguments={"yolo_model_path": "best-detect.pt"}.items(),
            ),
        ]
    )
