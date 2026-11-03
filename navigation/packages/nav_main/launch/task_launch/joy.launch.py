"""Publishes /joy from the connected gamepad (DualShock/DualSense).

Needed by demo_roborregosday.py and wait_for_controller_input_node.py.
Usage: ros2 launch nav_main joy.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"device_id": 0, "deadzone": 0.05}],
            )
        ]
    )
