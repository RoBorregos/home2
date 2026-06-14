#!/usr/bin/env python3
"""LOCALIZATION-ONLY for the mecanum omnibase — slam_toolbox against a saved map.

Competition-day path: map the arena ONCE with slam.launch.py, serialize it, then
run this every round. slam_toolbox loads the saved pose-graph, localizes against
it (replacing AMCL), and publishes the map->odom TF. It does NOT grow the map.

Brings up:
    slam_toolbox (localization)  ──>  TF map->odom   (localizes on the saved map)
    [optional] ekf               ──>  TF odom->base_link
    [optional] dashboard         ──>  /odrive/odom, /odrive/imu

NOT started here: the RPLIDAR C1 driver (/scan in base_link), run separately.

Prerequisite — a SERIALIZED map (.posegraph + .data), made while mapping:
    ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph "{filename: '/home/<user>/omnibase_maps/arena'}"

Usage
-----
    ros2 launch odrive_comm localization.launch.py \
        map:=/home/<user>/omnibase_maps/arena use_dashboard:=true

The robot's start pose on the map is set by map_start_pose in
mapper_params_localization.yaml (default [0,0,0]); override it there, or comment it
out and drop a "2D Pose Estimate" in RViz (publishes /initialpose).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('odrive_comm')
    default_params = os.path.join(pkg_share, 'config', 'mapper_params_localization.yaml')

    use_ekf = LaunchConfiguration('use_ekf')
    use_dashboard = LaunchConfiguration('use_dashboard')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_name = LaunchConfiguration('map')

    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='Also start the EKF (ekf.launch.py) for the odom->base_link TF. '
                    'Set false if you already run the EKF separately.')
    declare_use_dashboard = DeclareLaunchArgument(
        'use_dashboard', default_value='true',
        description='Forwarded to ekf.launch.py: also start odrive_dashboard (serial bridge + web GUI).')
    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='cmd_vel',
        description='Forwarded to ekf.launch.py: topic the dashboard listens on for velocity. '
                    'Defaults to "cmd_vel_smoothed" here because this launch is the navigation path '
                    '(nav2 velocity_smoother output). Set "cmd_vel" if you run nav2 without the smoother.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock instead of wall time (true only for rosbag replay in sim time).')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='slam_toolbox localization parameter YAML.')
    declare_map = DeclareLaunchArgument(
        'map', default_value='/workspace/mapa/.posegraph',
        description='Serialized pose-graph map to localize against — absolute path '
                    'WITHOUT extension (loads <map>.posegraph + <map>.data). '
                    'Overrides map_file_name from the params YAML.')

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('odrive_comm'), 'launch', 'ekf.launch.py'])),
        condition=IfCondition(use_ekf),
        launch_arguments={
            'use_dashboard': use_dashboard,
            'cmd_vel_topic': cmd_vel_topic,
        }.items(),
    )

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
        declare_use_ekf,
        declare_use_dashboard,
        declare_cmd_vel_topic,
        declare_use_sim_time,
        declare_params_file,
        declare_map,
        ekf_launch,
        slam_node,
    ])
