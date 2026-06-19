#!/usr/bin/env python3
"""LOCALIZATION-ONLY for the mecanum omnibase — slam_toolbox against a saved map.

Competition-day path: map the arena ONCE with slam.launch.py, serialize it, then
run this every round. slam_toolbox loads the saved pose-graph, localizes against
it (replacing AMCL), and publishes the map->odom TF. It does NOT grow the map.

Brings up:
    slam_toolbox (localization)  ──>  TF map->odom  +  /map  (localizes on the saved map)

NOT started here (owned by omni_basics.launch.py / run separately):
    * odrive_dashboard + EKF (odom->base_link)
    * the RPLIDAR C1 driver (/scan in base_link)

Prerequisite — a SERIALIZED map (.posegraph + .data), made while mapping:
    ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph "{filename: '/home/<user>/omnibase_maps/arena'}"

Usage
-----
    ros2 launch nav_main localization.launch.py map:=/home/<user>/omnibase_maps/arena

The robot's start pose on the map is set by map_start_pose in
mapper_params_localization.yaml (default [0,0,0]); override it there, or comment it
out and drop a "2D Pose Estimate" in RViz (publishes /initialpose).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_main')
    default_params = os.path.join(pkg_share, 'config','omni_config', 'mapper_params_localization.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_name = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock instead of wall time (true only for rosbag replay in sim time).')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='slam_toolbox localization parameter YAML.')
    declare_map = DeclareLaunchArgument(
        'map', default_value='/workspace/mapa',
        description='Serialized pose-graph map to localize against — absolute path '
                    'WITHOUT extension (loads <map>.posegraph + <map>.data). '
                    'Overrides map_file_name from the params YAML.')


    # Dedicated localization node: rolling-buffer scan match against the loaded
    # graph, publishes map->odom. The map arg overrides map_file_name in the YAML.
    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_name,
            },
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map,
        slam_node,
    ])
