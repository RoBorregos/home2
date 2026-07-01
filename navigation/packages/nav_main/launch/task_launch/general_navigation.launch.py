import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_function(context, *args, **kwargs):
    from frida_constants.navigation_constants import RTAB_MAPS_PATH, RETREAT_DISTANCE

    pkg_file_route = get_package_share_directory('nav_main')
    rtab_params_file = os.path.join(pkg_file_route, 'config', 'rtabmap', 'rtabmap_localization_config.yaml')
    nav2_params_file = os.path.join(pkg_file_route, 'config', 'nav2_standard.yaml')
    # --- LIMP 3-WHEEL MODE (rear-left ODrive / node 33 dead) -----------------
    # The omnibase Nav2 defaults to the crawl-speed limp profile while the base
    # runs on 3 wheels. Override at runtime with nav2_omni_config_file:=/path.yaml,
    # or revert this default to 'nav2_omni.yaml' once the ODrive is replaced.
    nav2_omni_limp_file = os.path.join(pkg_file_route, 'config', 'omni_config', 'nav2_omni_limp.yaml')

    rtabmap_map_name = LaunchConfiguration('map_name', default=os.getenv('MAP_NAME'))
    rtab_params = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    nav2_params = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    nav2_omni_params = LaunchConfiguration('nav2_omni_config_file', default=nav2_omni_limp_file)
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

    # Editable static map (Option A): serve <name>.yaml via a map_server inside the
    # nav2 container (so the .pgm can be hand-edited in the map tagger) instead of the
    # grid slam_toolbox rasterizes from the pose-graph. Auto-enabled when the grid
    # yaml exists next to the posegraph; override with use_static_map_server:=true|false.
    omni_map_yaml_default = omni_map_default + '.yaml'
    omni_map_yaml = LaunchConfiguration('map_yaml', default=omni_map_yaml_default)
    omni_map_yaml_value = omni_map_yaml.perform(context)
    use_static_map_default = 'true' if os.path.exists(omni_map_yaml_value) else 'false'
    use_static_map = LaunchConfiguration('use_static_map_server', default=use_static_map_default)
    static_on = use_static_map.perform(context).lower() in ('true', '1')
    print(f"[general_navigation] static map '{omni_map_yaml_value}' "
          f"exists={os.path.exists(omni_map_yaml_value)} -> use_static_map_server={use_static_map_default}")

    # Keepout (virtual obstacle) filter — AUTO-enabled when a mask named
    # "<MAP_NAME>_keepout_mask.yaml" exists next to the map (draw it in the map
    # tagger UI). Override with use_keepout:=true|false or keepout_mask:=/path.yaml.
    # Skipped while the static map server owns /map: with the editable .pgm you paint
    # obstacles straight into the map, so the keepout filter is redundant there (and it
    # avoids the extra filter servers/topics). Force it back on with use_keepout:=true.
    keepout_mask_default = os.path.join(maps_dir, f'{areas_map_name}_keepout_mask.yaml')
    keepout_mask = LaunchConfiguration('keepout_mask', default=keepout_mask_default)
    keepout_mask_value = keepout_mask.perform(context)
    keepout_exists = os.path.exists(keepout_mask_value)
    use_keepout_default = 'true' if (keepout_exists and not static_on) else 'false'
    use_keepout = LaunchConfiguration('use_keepout', default=use_keepout_default)
    if keepout_exists and static_on:
        print(f"[general_navigation] keepout mask '{keepout_mask_value}' exists but "
              f"use_static_map_server=true -> keepout DISABLED "
              f"(paint obstacles into the static .pgm; use_keepout:=true to force on)")
    else:
        print(f"[general_navigation] keepout mask '{keepout_mask_value}' "
              f"exists={keepout_exists} -> use_keepout={use_keepout_default}")

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
            'use_static_map_server': use_static_map,
        }.items(),
    )

    nav2_omni = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "omni_setup", "nav2_omni.launch.py"])
        ),
        launch_arguments={
            'nav2': nav2_activate,
            'nav2_config_file': nav2_omni_params,
            'use_keepout': use_keepout,
            'keepout_mask': keepout_mask,
            'use_static_map_server': use_static_map,
            'map_yaml': omni_map_yaml,
        }.items(),
    )

    # Table/shelf docking — perpendicular approach using the holonomic base + the
    # lidar/cloud. nav_central calls its undock service before each new goal.
    table_docker = Node(
        package='nav_main',
        executable='table_docker.py',
        name='table_docker',
        output='screen',
        parameters=[{'retreat_distance': RETREAT_DISTANCE}],
    )

    # Person-following bridge: forwards the vision tracker's target to the Nav2
    # GoalUpdater and switches nav2 between the standard/follow param sets when
    # nav_central calls /navigation/set_follow_mode. Idle until follow is requested,
    # so it is safe to run for every task (gpsr/ppc/dlc/hric).
    person_goal_smoother_node = Node(
        package='nav_main',
        executable='person_goal_smoother.py',
        name='person_goal_smoother',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'default_base': default_base,
        }],
    )

    launch_actions = [
        nav_central_node,
        nav_ui_node,
        person_goal_smoother_node,
    ]

    if default_base_value != 'omnibase':
        launch_actions.append(nav_basics)
        launch_actions.append(rtabmapnav)
    else:
        launch_actions.append(omni_basics)
        if nav_type_value == '2d':
            launch_actions.append(omni_localization)
        print(f"[general_navigation] omnibase Nav2 config -> "
              f"{nav2_omni_params.perform(context)}")
        launch_actions.append(nav2_omni)
        launch_actions.append(table_docker)

    return launch_actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_function)])
