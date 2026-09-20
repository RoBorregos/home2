#!/usr/bin/env python3
"""MoveIt for FRIDA on gz_ros2_control (robot_state_publisher and controllers come from sim.launch.py)."""

import yaml
from frida_gz_sim.description import XARM_ARGS
from frida_gz_sim.moveit_config import build_moveit_config
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    show_rviz = LaunchConfiguration("show_rviz")
    moveit_config = build_moveit_config(context)

    moveit_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_pkg"), "launch", "frida_moveit_common.launch.py"]
            )
        ),
        launch_arguments={
            "prefix": "",
            "attach_to": XARM_ARGS["attach_to"],
            "attach_xyz": '"0 0 0"',
            "attach_rpy": '"0 0 1.5707963267948966"',
            "no_gui_ctrl": "false",
            "show_rviz": show_rviz,
            "use_sim_time": "true",
            "moveit_config_dump": yaml.dump(moveit_config.to_dict()),
        }.items(),
    )

    downsample_pcd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("perception_3d"), "launch", "downsample_pc.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return [moveit_common, downsample_pcd]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("show_rviz", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
