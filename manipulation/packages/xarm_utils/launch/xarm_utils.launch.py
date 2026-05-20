from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    grasp_detector_launch = Node(
        package="xarm_utils",
        executable="grasp_detector.py",
        name="grasp_detector",
        output="screen",
        emulate_tty=True,
        parameters=[],
    )

    return LaunchDescription([grasp_detector_launch])
