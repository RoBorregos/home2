#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from frida_constants.manipulation_constants import (
    ZED_POINT_CLOUD_TOPIC,
)


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic", default=ZED_POINT_CLOUD_TOPIC)
    return LaunchDescription(
        [
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
                    }
                ],
            ),
        ]
    )
