from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="task_manager",
                executable="follow_face_node.py",
                name="follow_face_node",
                output="screen",
                emulate_tty=True,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("xarm_api"),
                            "launch",
                            "xarm6_driver.launch.py",
                        ]
                    )
                ),
                launch_arguments={"robot_ip": "192.168.31.180"}.items(),
            ),
        ]
    )
