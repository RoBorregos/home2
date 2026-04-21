#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # When the sim includes this launch it overrides use_sim_time to true so
    # gpd_service, manipulation_core, pick/place/pour servers and perception
    # all look up TFs against /clock. On the real robot the default (false)
    # keeps everything on wall time -- same launch, no sim-specific code.
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    sim_time_param = {"use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (true) for the MuJoCo sim, wall time (false) for real robot.",
            ),
            # gpd
            Node(
                package="arm_pkg",
                executable="gpd_service",
                name="gpd_service",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[sim_time_param],
            ),
            Node(
                package="pick_and_place",
                executable="manipulation_core.py",
                name="manipulation_core",
                output="screen",
                emulate_tty=True,
                parameters=[sim_time_param],
            ),
            DeclareLaunchArgument(
                "pick_min_height",
                default_value="0.1",
                description="Minimum height (m) a grasp pose must clear above the detected support plane. Real robot 0.1; sim lowers this because the plane segmenter tends to pick a wrong plane off the house mesh.",
            ),
            Node(
                package="pick_and_place",
                executable="pick_server.py",
                name="pick_server",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        # based on distance between end-effector link and contact point with objects e.g. where you grip
                        "ee_link_offset": -0.09,
                        "pick_min_height": LaunchConfiguration("pick_min_height"),
                    },
                    sim_time_param,
                ],
            ),
            Node(
                package="pick_and_place",
                executable="place_server.py",
                name="place_server",
                output="screen",
                emulate_tty=True,
                parameters=[sim_time_param],
            ),
            Node(
                package="pick_and_place",
                executable="pour_server.py",
                name="pour_server",
                output="screen",
                emulate_tty=True,
                parameters=[sim_time_param],
            ),
            # perception_3d.launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perception_3d"),
                            "launch",
                            "perception_3d.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "point_cloud_topic": LaunchConfiguration(
                        "point_cloud_topic", default="/point_cloud"
                    ),
                }.items(),
            ),
            Node(
                package="place",
                executable="heatmapPlace_Server.py",
                parameters=[sim_time_param],
            ),
            Node(
                package="frida_motion_planning",
                executable="motion_planning_server.py",
                parameters=[sim_time_param],
            ),
            Node(
                package="pick_and_place",
                executable="fix_position_to_plane.py",
                name="fix_position_to_plane",
                output="screen",
                emulate_tty=True,
                parameters=[sim_time_param],
            ),
        ]
    )
