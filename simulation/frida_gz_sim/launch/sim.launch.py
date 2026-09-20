#!/usr/bin/env python3
"""Gazebo world + FRIDA + ros2_control + sensor bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from frida_gz_sim.description import build_robot_description
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resource_path() -> str:
    """Model and mesh lookup path for gz (package models plus every package share)."""
    paths = [
        os.path.join(get_package_share_directory("frida_gz_sim"), "models"),
        os.path.expanduser("~/.gz/sim_models"),
    ]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        share = os.path.join(prefix, "share")
        if os.path.isdir(share):
            paths.append(share)
    extra = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if extra:
        paths.append(extra)
    return os.pathsep.join(paths)


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("frida_gz_sim")
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    world = LaunchConfiguration("world").perform(context)
    if not os.path.isabs(world):
        world = os.path.join(share, "worlds", world)
    width = int(LaunchConfiguration("image_width").perform(context))
    height = int(LaunchConfiguration("image_height").perform(context))
    rate = int(LaunchConfiguration("camera_rate").perform(context))
    grasp_assist = (
        LaunchConfiguration("grasp_assist").perform(context).lower() == "true"
    )

    robot_description = build_robot_description(
        image_width=width, image_height=height, camera_rate=rate
    )
    sim_time = {"use_sim_time": True}

    gz_cmd = ["gz", "sim", "-r", "-v", "3", world]
    if not gui:
        gz_cmd[2:2] = ["-s", "--headless-rendering"]
    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": _resource_path()},
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, sim_time],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "frida", "-z", "0.0"],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {"config_file": os.path.join(share, "config", "bridge.yaml")},
            sim_time,
        ],
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "180",
            ],
            parameters=[sim_time],
        )

    controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                spawner("joint_state_broadcaster"),
                spawner("xarm6_traj_controller"),
                spawner("xarm_gripper_traj_controller"),
            ],
        )
    )

    cloud_fix = Node(
        package="frida_gz_sim",
        executable="cloud_frame_fix.py",
        output="screen",
        parameters=[sim_time],
    )

    xarm_bridge = Node(
        package="frida_gz_sim",
        executable="xarm_sim_bridge.py",
        output="screen",
        parameters=[sim_time],
    )

    grasp_attach = Node(
        package="frida_gz_sim",
        executable="grasp_attach.py",
        output="screen",
        parameters=[sim_time],
    )

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _resource_path()),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        controllers,
        cloud_fix,
        xarm_bridge,
    ] + ([grasp_attach] if grasp_assist else [])


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("world", default_value="pnp_table.sdf"),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="360"),
            DeclareLaunchArgument("camera_rate", default_value="10"),
            # Weld grasped objects to the gripper (physics-only pinches slip in Gazebo)
            DeclareLaunchArgument("grasp_assist", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
