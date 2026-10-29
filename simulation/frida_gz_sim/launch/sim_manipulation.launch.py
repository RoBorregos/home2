#!/usr/bin/env python3
"""Real manipulation stack (MoveIt + pick_and_place) on sim time."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare

OCTOMAP_CLOUD_TOPIC = "/sim/octomap_cloud"


def isolate_remaps(context, *args, **kwargs):
    """Give this scope its own remap list.

    launch_ros' SetRemap appends to the list it inherited from the parent scope, so
    without this copy the remap below would also reach pick_and_place and leave the
    real /point_cloud without a publisher.
    """
    context.launch_configurations["ros_remaps"] = list(
        context.launch_configurations.get("ros_remaps", [])
    )
    return []


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("show_rviz", default_value="false"),
            # pick_and_place.launch.py no longer takes a use_sim_time argument, so set it
            # for every node launched from here; they must read TF against /clock
            SetParameter(name="use_sim_time", value=True),
            Node(
                package="frida_gz_sim",
                executable="octomap_cloud_filter.py",
                output="screen",
            ),
            # Publishes /point_cloud, so it stays outside the remapped scope below
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perception_3d"),
                            "launch",
                            "downsample_pc.launch.py",
                        ]
                    )
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            GroupAction(
                [
                    OpaqueFunction(function=isolate_remaps),
                    # Only MoveIt's octomap gets the filtered cloud; GPD keeps the full one
                    SetRemap(src="/point_cloud", dst=OCTOMAP_CLOUD_TOPIC),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("frida_gz_sim"),
                                    "launch",
                                    "sim_moveit.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "show_rviz": LaunchConfiguration("show_rviz")
                        }.items(),
                    ),
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("pick_and_place"),
                            "launch",
                            "pick_and_place.launch.py",
                        ]
                    )
                ),
            ),
        ]
    )
