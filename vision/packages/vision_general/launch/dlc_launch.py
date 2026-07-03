"""Vision launch for the Doing Laundry task.

Brings up the object detector include (detect_objects for the basket scan;
also starts image_orienter) PLUS the washing-machine insert-and-pick vision
chain: moondream_node (MoondreamObjectBBox for the drum opening) and
trash_detection_node (serves MOONDREAM_POINT_3D_TOPIC — bbox -> depth-ROI
mean 3D centroid). --dlc must enable the "moondream" compose profile
(docker/vision/run.sh) so the gRPC moondream server is up.
"""

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
            Node(
                package="vision_general",
                executable="trash_detection_node.py",
                name="trash_detection_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
