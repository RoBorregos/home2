"""HRI follow-person bringup — OMNIBASE by default (dashgo still selectable).

Same base-selection convention as general_navigation.launch.py, plus the
person_goal_smoother that bridges the vision tracker to the Nav2 GoalUpdater.

  omnibase (default): omni_basics + slam_toolbox localization + nav2_omni
  dashgo            : nav_basics + RTABMap localization + nav2 (nav2_standard)

The actual "follow person" trigger is the /navigation/follow_person service on
nav_central; it switches person_goal_smoother to follow mode (which swaps in the
nav2_omni_following.yaml params) and sends a NavigateToPose goal with the
follow_dynamic_point.xml behaviour tree.

Override the base:   ros2 launch nav_main hric.launch.py default_base:=dashgo
"""

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
    # Active omnibase Nav2 profile: 3-WHEEL LIMP (base running degraded, one
    # wheel out — see nav2_omni_limp.yaml header). Single source for BOTH the
    # nav2_omni include and the person_goal_smoother restore pair below; point
    # back at nav2_omni.yaml once the base is healthy again.
    nav2_omni_file = os.path.join(pkg_file_route, 'config', 'omni_config', 'nav2_omni_limp.yaml')
    nav2_omni_follow_file = nav2_omni_file.replace('.yaml', '_following.yaml')

    rtabmap_map_name = LaunchConfiguration('map_name', default=os.getenv('MAP_NAME'))
    rtab_params = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    nav2_params = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    localization = LaunchConfiguration('localization', default='true')
    nav2_activate = LaunchConfiguration('nav2', default='true')

    # Values to select base (same convention as general_navigation.launch.py)
    default_base = LaunchConfiguration('default_base', default='omnibase')  # Other option "dashgo"
    default_base_value = default_base.perform(context)
    nav_type = LaunchConfiguration('nav_type', default='2d')  # Other 3d
    nav_type_value = nav_type.perform(context)

    areas_map_name = context.perform_substitution(rtabmap_map_name).replace('.db', '')

    # slam_toolbox serialized map for the omnibase localization (absolute path,
    # WITHOUT extension) — same location nav_ui saves it to.
    nav_src = os.path.dirname(os.path.normpath(RTAB_MAPS_PATH))
    maps_dir = os.path.join(nav_src, 'packages', 'map_context', 'maps')
    omni_map_default = os.path.join(maps_dir, areas_map_name)
    omni_map = LaunchConfiguration('map', default=omni_map_default)

    # Keepout (virtual obstacle) filter — AUTO-enabled when a mask named
    # "<MAP_NAME>_keepout_mask.yaml" exists next to the map.
    keepout_mask_default = os.path.join(maps_dir, f'{areas_map_name}_keepout_mask.yaml')
    keepout_mask = LaunchConfiguration('keepout_mask', default=keepout_mask_default)
    keepout_mask_value = keepout_mask.perform(context)
    use_keepout_default = 'true' if os.path.exists(keepout_mask_value) else 'false'
    use_keepout = LaunchConfiguration('use_keepout', default=use_keepout_default)
    print(f"[hric] keepout mask '{keepout_mask_value}' "
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

    # Bridges the vision tracker -> Nav2 GoalUpdater, and switches nav2 between the
    # standard and follow param sets for the active base (default_base picks the
    # config pair: omnibase -> nav2_omni(_following).yaml, dashgo -> nav2_(standard|following).yaml).
    smoother_params = {'default_base': default_base}
    if default_base_value == 'omnibase':
        # Keep the smoother's restore pair in sync with the config Nav2 is
        # launched with below — otherwise deactivating follow mode would
        # re-apply healthy nav2_omni.yaml speeds onto the limp base.
        smoother_params['standard_config_file'] = nav2_omni_file
        smoother_params['follow_config_file'] = nav2_omni_follow_file
    person_goal_smoother_node = Node(
        package='nav_main',
        executable='person_goal_smoother.py',
        name='person_goal_smoother',
        output='screen',
        emulate_tty=True,
        parameters=[smoother_params],
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
            'nav2_config_file': nav2_omni_file,
            'use_keepout': use_keepout,
            'keepout_mask': keepout_mask,
        }.items(),
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
        launch_actions.append(nav2_omni)

    return launch_actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_function)])
