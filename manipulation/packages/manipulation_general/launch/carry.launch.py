#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    input_topic = LaunchConfiguration(
        "input_topic", default="/zed/zed_node/point_cloud/cloud_registered"
    )
    return LaunchDescription(
        [
            # perception_3d.launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("arm_pkg"),
                            "launch",
                            "frida_moveit_config.launch.py",
                        ]
                    )
                ),
            ),
            Node(
                package="frida_motion_planning",
                executable="motion_planning_server.py",
            ),
            Node(
                package="task_manager",
                executable="follow_person_node.py",
                name="follow_person_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="perception_3d",
                executable="down_sample_pc",
                name="downsample_pc_2",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "input_topic": input_topic,
                        "small_size": 0.03,
                        "medium_size": 0.03,
                        "large_size": 0.03,
                        "OutputPointCloudTopic": "/point_cloud_nav",
                    }
                ],
            ),
        ]
    )
