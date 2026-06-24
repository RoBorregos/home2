from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motion_planning_launch = Node(
        package="frida_motion_planning",
        executable="motion_planning_server.py",
        name="motion_planning_server",
    )
    align_arm_launch = Node(
        package="frida_motion_planning",
        executable="align_arm_server.py",
        name="align_arm_server",
    )

    return LaunchDescription([motion_planning_launch, align_arm_launch])
