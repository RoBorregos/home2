#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
import math
import os


def construct_angle_radians(loader, node):
    """Utility function to construct radian values from yaml."""
    value = loader.construct_scalar(node)
    try:
        return float(value)
    except SyntaxError:
        raise Exception("invalid expression: %s" % value)


def construct_angle_degrees(loader, node):
    """Utility function for converting degrees into radians from yaml."""
    return math.radians(construct_angle_radians(loader, node))


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        yaml.SafeLoader.add_constructor("!radians", construct_angle_radians)
        yaml.SafeLoader.add_constructor("!degrees", construct_angle_degrees)
    except Exception:
        raise Exception("yaml support not available; install python-yaml")

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix", default="")
    attach_to = LaunchConfiguration("attach_to", default="world")
    attach_xyz = LaunchConfiguration("attach_xyz", default='"0 0 0"')
    attach_rpy = LaunchConfiguration("attach_rpy", default='"0 0 0"')
    no_gui_ctrl = LaunchConfiguration("no_gui_ctrl", default=False)
    show_rviz = LaunchConfiguration("show_rviz", default=True)
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    moveit_config_dump = LaunchConfiguration("moveit_config_dump")

    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader)
    moveit_config_package_name = "xarm_moveit_config"

    octomap_config = {
        "octomap_frame": "base",
        "octomap_resolution": 0.01,
        "max_range": 2.0,
    }

    octomap_updater_config = load_yaml("arm_pkg", "config/sensors_3d.yaml")

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config_dict,
            {"use_sim_time": use_sim_time},
            octomap_config,
            octomap_updater_config,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package_name),
            "rviz",
            "planner.rviz" if no_gui_ctrl.perform(context) == "true" else "moveit.rviz",
        ]
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            {
                "robot_description": moveit_config_dict["robot_description"],
                "robot_description_semantic": moveit_config_dict[
                    "robot_description_semantic"
                ],
                "robot_description_kinematics": moveit_config_dict[
                    "robot_description_kinematics"
                ],
                "robot_description_planning": moveit_config_dict[
                    "robot_description_planning"
                ],
                "planning_pipelines": moveit_config_dict["planning_pipelines"],
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(show_rviz),
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    xyz = attach_xyz.perform(context)[1:-1].split(" ")
    rpy = attach_rpy.perform(context)[1:-1].split(" ")
    args = (
        xyz
        + rpy
        + [attach_to.perform(context), "{}link_base".format(prefix.perform(context))]
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=args,
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot_planner_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("xarm_planner"), "launch", "_robot_planner.launch.py"]
            )
        ),
        condition=IfCondition(no_gui_ctrl),
        launch_arguments={
            "moveit_config_dump": moveit_config_dump,
        }.items(),
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz2_node, on_exit=[EmitEvent(event=Shutdown())]
            )
        ),
        rviz2_node,
        static_tf,
        move_group_node,
        robot_planner_node_launch,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
