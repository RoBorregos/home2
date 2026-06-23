import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_function(context, *args, **kwargs):
    from frida_constants.navigation_constants import RTAB_MAPS_PATH

    pkg_file_route = get_package_share_directory('nav_main')
    rtab_params_file = os.path.join(pkg_file_route, 'config', 'rtabmap', 'rtabmap_localization_config.yaml')
    nav2_params_file = os.path.join(pkg_file_route, 'config', 'nav2_standard.yaml')

    rtabmap_map_name = LaunchConfiguration('map_name', default=os.getenv('MAP_NAME'))
    rtab_params = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    nav2_params = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    localization = LaunchConfiguration('localization', default='true')
    nav2_activate = LaunchConfiguration('nav2', default='true')

    # Values to select base (same convention as mapping.launch.py)
    default_base = LaunchConfiguration('default_base', default='omnibase')  # Other option "dashgo"
    default_base_value = default_base.perform(context)
    nav_type = LaunchConfiguration('nav_type', default='2d')  # Other 3d
    nav_type_value = nav_type.perform(context)

    areas_map_name = context.perform_substitution(rtabmap_map_name).replace('.db', '')

    # slam_toolbox serialized map for the omnibase localization (absolute path,
    # WITHOUT extension) — matches where nav_ui saves it:
    #   <workspace>/packages/map_context/maps/<name>(.posegraph + .data)
    nav_src = os.path.dirname(os.path.normpath(RTAB_MAPS_PATH))
    maps_dir = os.path.join(nav_src, 'packages', 'map_context', 'maps')
    omni_map_default = os.path.join(maps_dir, areas_map_name)
    omni_map = LaunchConfiguration('map', default=omni_map_default)

    # Keepout (virtual obstacle) filter — AUTO-enabled when a mask named
    # "<MAP_NAME>_keepout_mask.yaml" exists next to the map (draw it in the map
    # tagger UI). Override with use_keepout:=true|false or keepout_mask:=/path.yaml.
    keepout_mask_default = os.path.join(maps_dir, f'{areas_map_name}_keepout_mask.yaml')
    keepout_mask = LaunchConfiguration('keepout_mask', default=keepout_mask_default)
    keepout_mask_value = keepout_mask.perform(context)
    use_keepout_default = 'true' if os.path.exists(keepout_mask_value) else 'false'
    use_keepout = LaunchConfiguration('use_keepout', default=use_keepout_default)
    print(f"[general_navigation] keepout mask '{keepout_mask_value}' "
          f"exists={os.path.exists(keepout_mask_value)} -> use_keepout={use_keepout_default}")

    nav_central_node = Node(
        package='nav_main',
        executable='nav_central.py',
        name='nav_central',
        namespace='',
        output='screen',
        parameters=[{
            'localization': localization,
            'map_name': rtabmap_map_name,
            'areas_map_name': areas_map_name,
            'rtab_localization_config': rtab_params,
            'rtab_mapping_config': rtab_params,
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
            'map_name': rtabmap_map_name,
            'default_base': default_base,
            'nav_type': nav_type,
        }],
    )

    # ----- dashgo base: RTABMap RGBD localization + nav2 -----
    nav_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "dashgo_base", "nav_basics.launch.py"])
        ),
    )

    rtabmapnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "dashgo_base", "rtabnav2.launch.py"])
        ),
        launch_arguments={
            'localization': localization,
            'rtab_config_file': rtab_params,
            'nav2_config_file': nav2_params,
            'nav2': nav2_activate,
            'map_name': rtabmap_map_name,
        }.items(),
    )

    # ----- omnibase: slam_toolbox localization + nav2_omni -----
    omni_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "omni_setup", "omni_basics.launch.py"])
        ),
    )

    omni_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "omni_setup", "localization.launch.py"])
        ),
        launch_arguments={
            'map': omni_map,
        }.items(),
    )

    nav2_omni = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "omni_setup", "nav2_omni.launch.py"])
        ),
        launch_arguments={
            'nav2': nav2_activate,
            'use_keepout': use_keepout,
            'keepout_mask': keepout_mask,
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

    # Approach-direction worker (strafe/drive in a commanded direction to N cm).
    # Omnibase-only, same as table_docker (left/right need holonomic strafing).
    approach_direction = Node(
        package='nav_main',
        executable='approach_direction.py',
        name='approach_direction',
        output='screen',
    )

    launch_actions = [
        nav_central_node,
        nav_ui_node,
    ]

    if default_base_value != 'omnibase':
        launch_actions.append(nav_basics)
        launch_actions.append(rtabmapnav)
    else:
        launch_actions.append(omni_basics)
        if nav_type_value == '2d':
            launch_actions.append(omni_localization)
        launch_actions.append(nav2_omni)
        launch_actions.append(table_docker)
        launch_actions.append(approach_direction)

    return launch_actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_function)])
