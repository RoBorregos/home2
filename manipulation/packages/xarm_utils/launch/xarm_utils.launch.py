
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ir_gripper_launch = Node(
        package="xarm_utils",
        executable="ir_gripper.py",
        name="ir_gripper",
        output="screen",
        emulate_tty=True,
        parameters=[],
    )

    return LaunchDescription([ir_gripper_launch])
