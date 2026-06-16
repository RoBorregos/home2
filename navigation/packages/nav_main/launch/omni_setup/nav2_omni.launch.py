from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import tempfile


def _deep_merge(base, overlay):
    """Recursively merge overlay dict into base dict (overlay wins)."""
    for k, v in overlay.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def launch_setup(context, *args, **kwargs):
    from frida_constants.navigation_constants import RTAB_MAPS_PATH

    pkg_file_route = get_package_share_directory('nav_main')
    nav2_params_file = os.path.join(pkg_file_route, 'config', 'omni_config', 'nav2_omni.yaml')
    keepout_overlay_file = os.path.join(pkg_file_route, 'config', 'omni_config', 'nav2_omni_keepout.yaml')
    nav2_activate = LaunchConfiguration('nav2', default='true')

    nav2_cfg_path = LaunchConfiguration('nav2_config_file', default=nav2_params_file).perform(context)

    # Keepout (virtual obstacle) filter — opt-in.
    use_keepout = LaunchConfiguration('use_keepout', default='false').perform(context)
    keepout_on = use_keepout.lower() in ('true', '1')
    nav_src = os.path.dirname(os.path.normpath(RTAB_MAPS_PATH))
    keepout_mask_default = os.path.join(
        nav_src, 'packages', 'map_context', 'maps', 'keepout_mask.yaml')
    keepout_mask = LaunchConfiguration('keepout_mask', default=keepout_mask_default)

    # Build the nav2 param file. When keepout is on, deep-merge the keepout overlay
    # into the nav2 params and write ONE merged file. Passing a single file is the
    # reliable way to get the filter into the composed costmap sub-nodes — extra
    # param files do not merge cleanly there. When off, use the base params as-is,
    # so the costmaps never load the filter (no "mask not received" spam).
    params_path = nav2_cfg_path
    if keepout_on:
        with open(nav2_cfg_path) as f:
            merged = yaml.safe_load(f)
        with open(keepout_overlay_file) as f:
            _deep_merge(merged, yaml.safe_load(f))
        params_path = os.path.join(tempfile.gettempdir(), 'nav2_omni_keepout_merged.yaml')
        with open(params_path, 'w') as f:
            yaml.safe_dump(merged, f, default_flow_style=False, sort_keys=False)

    nav2_params = ParameterFile(params_path, allow_substs=True)

    log_output = 'own_log' if os.getenv('NAV_QUIET') == '1' else 'screen'

    # Container 2: Nav2 — empty container, nodes loaded after startup
    nav2_container = Node(
        condition=IfCondition(nav2_activate),
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output=log_output,
        parameters=[nav2_params],
    )

    # Nav2 composable nodes loaded after container is confirmed running
    nav2_nodes = LoadComposableNodes(
        condition=IfCondition(nav2_activate),
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[nav2_params],
            ),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[nav2_params],
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[nav2_params],
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[nav2_params],
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[nav2_params],
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[nav2_params],
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': False,
                    'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'velocity_smoother',
                    ]
                }],
            ),
        ],
    )

    # Delay node loading until container process is running
    delayed_nav2_load = RegisterEventHandler(
        condition=IfCondition(nav2_activate),
        event_handler=OnProcessStart(
            target_action=nav2_container,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[nav2_nodes]
                )
            ]
        )
    )

    actions = [nav2_container, delayed_nav2_load]

    # --- Keepout filter servers (only when use_keepout:=true) -----------------
    # filter_mask_server publishes the painted mask; costmap_filter_info_server
    # tells the KeepoutFilter how to read it; a dedicated lifecycle manager brings
    # them up (autostart) independently of nav_central's nav2 startup.
    if keepout_on:
        actions += [
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                output=log_output,
                parameters=[nav2_params, {'yaml_filename': keepout_mask}],
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output=log_output,
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_costmap_filters',
                output=log_output,
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
                }],
            ),
        ]

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
