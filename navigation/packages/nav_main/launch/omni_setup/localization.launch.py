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

    # Serialized pose-graph base path (no extension). The editable occupancy grid
    # defaults to "<map>.yaml" right next to it (that is what nav_ui saves).
    map_base = map_name.perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    if not map_yaml:
        map_yaml = map_base + '.yaml'

    # Auto-enable the editable static map server when the grid YAML exists; an
    # explicit use_static_map_server:=true|false overrides the auto-detection.
    use_static_arg = LaunchConfiguration('use_static_map_server').perform(context)
    if use_static_arg == '':
        static_on = os.path.exists(map_yaml)
    else:
        static_on = use_static_arg.lower() in ('true', '1')

    print(f"[localization] static map server: yaml '{map_yaml}' "
          f"exists={os.path.exists(map_yaml)} -> use_static_map_server={static_on}")

    # Dedicated localization node: rolling-buffer scan match against the loaded
    # graph, publishes map->odom. The map arg overrides map_file_name in the YAML.
    # When the editable static map is on, move slam_toolbox's rasterized grid off
    # /map (-> /slam_map) so map_server owns the /map the costmap static_layer reads.
    slam_remaps = [('/map', '/slam_map'), ('/map_metadata', '/slam_map_metadata')] if static_on else []
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

    actions = [slam_node]

    if static_on:
        # Serve the editable occupancy grid as the latched /map. Co-aligned with
        # the pose-graph because nav_ui exports both from the same SLAM session.
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'topic_name': 'map',
                'frame_id': 'map',
            }],
        )
        # Brings map_server through configure->activate (autostart) so /map is
        # published before the costmaps come up. Separate from nav2's own manager.
        lifecycle_manager_map = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        )
        actions += [map_server, lifecycle_manager_map]

    return actions


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
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml', default_value='',
        description='Occupancy-grid YAML served as the EDITABLE static /map via '
                    'map_server. Default: <map>.yaml next to the serialized map. '
                    'Edit its .pgm to add/erase obstacles in the planning grid.')
    declare_use_static = DeclareLaunchArgument(
        'use_static_map_server', default_value='',
        description='Serve <map>.yaml via map_server as /map instead of the grid '
                    'slam_toolbox rasterizes from the pose-graph. Auto-enabled when '
                    'the grid YAML exists; set true|false to override.')

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map,
        declare_map_yaml,
        declare_use_static,
        OpaqueFunction(function=launch_setup),
    ])
