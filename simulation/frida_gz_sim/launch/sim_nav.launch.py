#!/usr/bin/env python3
"""Real navigation stack (scan merger + slam_toolbox localization + nav2 + nav_central) on sim time.

sim.launch.py must be running with mobile_base:=true and the arena world, so that
/cmd_vel, /scan_front, /scan_rear, /odom and the odom -> base_link TF exist.
"""

import os

from ament_index_python.packages import get_package_share_directory
from frida_constants.navigation_constants import RETREAT_DISTANCE
from frida_gz_sim.nav import SPAWN_POSE
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("frida_gz_sim")
    nav_main = get_package_share_directory("nav_main")
    map_name = LaunchConfiguration("map_name").perform(context)
    maps_dir = os.path.join(get_package_share_directory("map_context"), "maps")
    map_path = os.path.join(maps_dir, map_name)
    nav2_config_file = LaunchConfiguration("nav2_config_file").perform(context)

    # Same merge the robot runs: two RPLIDARs -> one 360 deg /scan in base_link
    merger = Node(
        package="ira_laser_tools",
        executable="laserscan_multi_merger",
        name="laserscan_multi_merger",
        output="screen",
        parameters=[
            {
                "destination_frame": "base_link",
                "scan_destination_topic": "/scan",
                "cloud_destination_topic": "/merged_cloud",
                "laserscan_topics": "/scan_rear /scan_front",
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.012566,
                "scan_time": 0.1,
                "range_min": 0.05,
                "range_max": 12.0,
            }
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_main, "launch", "omni_setup", "localization.launch.py")
        ),
        launch_arguments={
            "map": map_path,
            "use_sim_time": "true",
            "params_file": os.path.join(share, "config", "mapper_params_sim.yaml"),
            "use_static_map_server": "true",
        }.items(),
    )

    # nav2_omni.launch.py defaults to nav2_omni_limp.yaml (3-wheel degraded profile)
    # when nav2_config_file is left unset, so leaving it out here keeps sim in sync
    # with whatever profile the real robot is currently running. Pass
    # nav2_config_file:=.../nav2_omni.yaml explicitly to test the healthy-base tuning.
    nav2_launch_arguments = {
        "nav2": "true",
        "nav2_overlay_file": os.path.join(share, "config", "nav2_sim_overlay.yaml"),
        "use_keepout": "false",
        "use_static_map_server": "true",
        "map_yaml": map_path + ".yaml",
    }
    if nav2_config_file:
        nav2_launch_arguments["nav2_config_file"] = nav2_config_file
    else:
        # An include shares this launch's configurations, so the declared empty default
        # would reach nav2_omni.launch.py and shadow its own nav2_omni_limp.yaml default
        context.launch_configurations.pop("nav2_config_file", None)

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_main, "launch", "omni_setup", "nav2_omni.launch.py")
        ),
        launch_arguments=nav2_launch_arguments.items(),
    )

    # nav2's lifecycle manager ignores the launch-wide sim time; fix it before it bonds
    nav2_sim_time = Node(
        package="frida_gz_sim",
        executable="nav2_sim_time.py",
        output="screen",
    )

    nav_central = Node(
        package="nav_main",
        executable="nav_central.py",
        name="nav_central",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "localization": True,
                "map_name": map_name,
                "areas_map_name": map_name,
                "default_base": "omnibase",
                "nav_type": "2d",
            }
        ],
    )

    # dock_table goes through table_docker, as in general_navigation.launch.py. The sim
    # has no /point_cloud and publishes odometry on /odom, so it docks off the merged scan
    table_docker = Node(
        package="nav_main",
        executable="table_docker.py",
        name="table_docker",
        output="screen",
        parameters=[
            {
                "retreat_distance": RETREAT_DISTANCE,
                "odom_topic": "/odom",
                "detect_source": "scan",
            }
        ],
    )

    # nav_central blocks until someone sets the start pose; in sim we already know it
    initial_pose = Node(
        package="frida_gz_sim",
        executable="initial_pose.py",
        output="screen",
        parameters=[{"x": SPAWN_POSE[0], "y": SPAWN_POSE[1], "yaw": SPAWN_POSE[2]}],
    )

    return [
        SetParameter(name="use_sim_time", value=True),
        merger,
        localization,
        nav2,
        nav2_sim_time,
        table_docker,
        # nav_central sends the nav2 STARTUP; give the lifecycle manager time first
        TimerAction(period=10.0, actions=[nav_central]),
        # After nav_central, so the pose is not delivered before the node that waits for it
        TimerAction(period=12.0, actions=[initial_pose]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_name", default_value="robocup2026_1"),
            DeclareLaunchArgument(
                "nav2_config_file",
                default_value="",
                description=(
                    "Override for nav2_omni.launch.py's nav2_config_file arg. Leave "
                    "empty to inherit its default (currently nav2_omni_limp.yaml, the "
                    "3-wheel limp profile the real robot is running)."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
