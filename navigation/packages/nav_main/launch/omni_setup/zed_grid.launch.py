#!/usr/bin/env python3
"""ZED depth -> 2D occupancy grid with RTAB-Map, alongside slam_toolbox (#1318).

The lidars sit at z=0.136 m, so table tops, chair seats and shelves never reach
the slam_toolbox map. This runs RTAB-Map as a grid-only mapper on the ZED depth,
using slam_toolbox's map->base_link as its pose (see rtabmap_grid_only.yaml).
The grid lands on /rtabmap/map, in the same coordinates as slam_toolbox's /map,
ready for map_context/scripts/merge_grids.py.

Requires slam_toolbox (slam.launch.py) running and the ZED publishing. The ZED is
on the gripper: keep the arm in the navigation pose while mapping.

Offline on a bag:
    ros2 launch nav_main slam.launch.py use_sim_time:=true
    ros2 launch nav_main zed_grid.launch.py use_sim_time:=true
    ros2 bag play <bag> --clock
    ros2 run nav2_map_server map_saver_cli -t /rtabmap/map -f /abs/path/zed_grid

Full RTAB-Map instead (Phase 2, no slam_toolbox running):
    ros2 launch nav_main zed_grid.launch.py params_file:=<.../rtabmap_mapping_config.yaml>
"""

import yaml
from frida_constants.navigation_constants import (
    CAMERA_DEPTH_TOPIC,
    CAMERA_INFO_TOPIC,
    CAMERA_RGB_TOPIC,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    with open(params_file) as f:
        cfg = yaml.safe_load(f)

    def params(node):
        return [{**cfg.get(node, {}).get('ros__parameters', {}), 'use_sim_time': use_sim_time}]

    # Namespace rtabmap: grid is /rtabmap/map, never slam_toolbox's /map.
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync',
        namespace='rtabmap', output='screen',
        parameters=params('rgbd_sync'),
        remappings=[
            ('rgb/image', CAMERA_RGB_TOPIC),
            ('rgb/camera_info', CAMERA_INFO_TOPIC),
            ('depth/image', CAMERA_DEPTH_TOPIC),
        ],
    )
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        namespace='rtabmap', output='screen',
        parameters=params('rtabmap'),
        remappings=[('scan', '/scan')],
    )
    return [rgbd_sync, rtabmap]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('nav_main'), 'config', 'rtabmap', 'rtabmap_grid_only.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='RTAB-Map YAML (rgbd_sync + rtabmap sections).'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when replaying a bag with --clock.'),
        OpaqueFunction(function=launch_setup),
    ])
