import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_function(context, *args, **kwargs):
    pkg_file_route = get_package_share_directory('nav_main')
    # Restaurant maps the arena live while serving, so use the mapping rtab config
    # and a dedicated session DB.
    rtab_params_file = os.path.join(
        pkg_file_route, 'config', 'rtabmap', 'rtabmap_mapping_config.yaml'
    )
    rtab_params = LaunchConfiguration('rtab_config_file', default=rtab_params_file)

    # Base / nav-type selection (same convention as general_navigation/mapping).
    default_base = 'omnibase'
    nav_type = LaunchConfiguration('nav_type', default='2d')  # other: 3d
    nav_type_value = nav_type.perform(context)

    rtabmap_map_name = LaunchConfiguration('map_name', default='restaurant_session.db')
    areas_map_name = context.perform_substitution(rtabmap_map_name).replace('.db', '')

    nav_central_node = Node(
        package='nav_main',
        executable='nav_central.py',
        name='nav_central',
        namespace='',
        output='screen',
        parameters=[{
            'mapping': True,
            'localization': False,
            'use_nav2': True,  # hybrid SLAM + nav: map live AND run nav2 (default is off in mapping mode)
            'map_name': rtabmap_map_name,
            'areas_map_name': areas_map_name,
            'rtab_mapping_config': rtab_params,
            'rtab_localization_config': rtab_params,
            'default_base': default_base,
            'nav_type': nav_type,
        }],
    )

    nav_ui_node = Node(
        package='map_context',
        executable='nav_ui.py',
        name='nav_ui',
        output='screen',
        parameters=[{
            'mode': 'mapping',
            'map_name': rtabmap_map_name,
            'default_base': default_base,
            'nav_type': nav_type,
        }],
    )

    # ----- omnibase: slam_toolbox MAPPING + nav2_omni + table docking -----
    # (restaurant maps while it serves, so use slam.launch.py — mapping — rather than
    #  localization.launch.py, but still bring up nav2_omni so it can navigate.)
    omni_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "omni_setup", "omni_basics.launch.py"])
        ),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "omni_setup", "slam.launch.py"])
        ),
    )

    # Restaurant deltas (slow velocities, longer voxel persistence for
    # camera-marked tabletops, wider inflation) deep-merged over nav2_omni.yaml.
    nav2_restaurant_overlay = os.path.join(
        pkg_file_route, 'config', 'omni_config', 'nav2_omni_restaurant.yaml'
    )
    nav2_omni = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "omni_setup", "nav2_omni.launch.py"])
        ),
        launch_arguments={
            'nav2': 'true',
            'nav2_overlay_file': nav2_restaurant_overlay,
        }.items(),
    )

    # Table/shelf docking — perpendicular approach using the holonomic base + the
    # lidar/cloud. nav_central calls its undock service before each new goal.
    table_docker = Node(
        package='nav_main',
        executable='table_docker.py',
        name='table_docker',
        output='screen',
    )

    launch_actions = [
        nav_central_node,
        nav_ui_node,
        omni_basics,
    ]
    if nav_type_value == '2d':
        launch_actions.append(slam_toolbox)
    launch_actions.append(nav2_omni)
    launch_actions.append(table_docker)

    return launch_actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_function)])
