#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from frida_constants.manipulation_constants import (
    ZED_POINT_CLOUD_TOPIC,
)


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic", default=ZED_POINT_CLOUD_TOPIC)
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (true) for the MuJoCo sim, wall time (false) for real robot.",
            ),
            Node(
                package="perception_3d",
                executable="down_sample_pc",
                # arg input_topic
                name="downsample_pc",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "input_topic": input_topic,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
