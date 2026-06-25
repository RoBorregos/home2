#!/usr/bin/env python3
"""LOCALIZATION-ONLY for the mecanum omnibase — slam_toolbox against a saved map.

Competition-day path: map the arena ONCE with slam.launch.py, serialize it, then
run this every round. slam_toolbox loads the saved pose-graph, localizes against
it (replacing AMCL), and publishes the map->odom TF. It does NOT grow the map.

Brings up:
    slam_toolbox (localization)  ──>  TF map->odom  (scan-match against the graph)
    map_server (when a grid YAML exists)  ──>  /map  (EDITABLE static occupancy grid)

Editable static map (Option A)
------------------------------
By default slam_toolbox rasterizes /map from the serialized pose-graph, so you
CANNOT hand-edit obstacles in it. Instead, when a "<map>.yaml" occupancy grid
exists next to the serialized map, this launch serves THAT grid on /map via
map_server and moves slam_toolbox's own grid to /slam_map. slam_toolbox still
does pose localization (map->odom TF); the costmap static_layer reads the file.
So to erase/add an obstacle: edit <map>.pgm (free=254, unknown=205, occupied=0),
keep <map>.yaml, relaunch. Disable with use_static_map_server:=false.

NOT started here (owned by omni_basics.launch.py / run separately):
    * odrive_dashboard + EKF (odom->base_link)
    * the RPLIDAR C1 driver (/scan in base_link)

Prerequisite — a SERIALIZED map (.posegraph + .data), made while mapping:
    ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph "{filename: '/home/<user>/omnibase_maps/arena'}"
    (nav_ui's save also exports <name>.yaml + <name>.pgm — the editable grid.)

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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_name = LaunchConfiguration('map')

    # When the editable static map server is active (map_server is loaded INSIDE the
    # nav2 container by nav2_omni.launch.py — co-located so the latched /map is
    # delivered in-process and not dropped), move slam_toolbox's own rasterized grid
    # off /map -> /slam_map so map_server owns the /map the costmap static_layer reads.
    # slam_toolbox keeps doing pose localization (map->odom TF) either way.
    static_on = LaunchConfiguration('use_static_map_server').perform(context).lower() in ('true', '1')
    slam_remaps = [('/map', '/slam_map'), ('/map_metadata', '/slam_map_metadata')] if static_on else []
    print(f"[localization] use_static_map_server={static_on} -> slam_toolbox "
          + ("/map remapped to /slam_map (map_server owns /map)" if static_on
             else "publishes /map directly"))

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
        remappings=slam_remaps,
    )

    return [slam_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_main')
    default_params = os.path.join(pkg_share, 'config', 'omni_config', 'mapper_params_localization.yaml')

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
    declare_use_static = DeclareLaunchArgument(
        'use_static_map_server', default_value='false',
        description='If true, slam_toolbox stops owning /map (remapped to /slam_map) '
                    'because map_server (loaded in the nav2 container by '
                    'nav2_omni.launch.py) serves the editable <map>.pgm on /map. '
                    'general_navigation.launch.py auto-sets this when <map>.yaml exists.')

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map,
        declare_use_static,
        OpaqueFunction(function=launch_setup),
    ])
