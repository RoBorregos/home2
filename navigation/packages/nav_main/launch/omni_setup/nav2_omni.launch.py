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
            # velocity_smoother removed: the base subscribes to raw /cmd_vel and does its
            # own acceleration ramp on the MCU, so the Nav2 smoother was bypassed (nothing
            # consumed /cmd_vel_smoothed). Dropping it also speeds up the lifecycle resume.
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
                    ]
                }],
            ),
        ],
    )

    # Actions to run once the container is up: the nav2 stack, plus (when enabled)
    # the keepout filter servers loaded INTO THE SAME CONTAINER as the costmaps.
    # Co-locating them keeps the latched filter-info/mask topics in the same
    # process as the costmap filters, which delivers reliably (cross-process
    # transient-local was being dropped — e.g. via shared memory).
    on_start_actions = [TimerAction(period=2.0, actions=[nav2_nodes])]

    keepout_lifecycle = None
    if keepout_on:
        keepout_nodes = LoadComposableNodes(
            target_container='nav2_container',
            composable_node_descriptions=[
                # filter_mask_server publishes the painted mask...
                ComposableNode(
                    package='nav2_map_server',
                    plugin='nav2_map_server::MapServer',
                    name='filter_mask_server',
                    parameters=[nav2_params, {'yaml_filename': keepout_mask}],
                ),
                # ...costmap_filter_info_server tells the KeepoutFilter how to read it.
                ComposableNode(
                    package='nav2_map_server',
                    plugin='nav2_map_server::CostmapFilterInfoServer',
                    name='costmap_filter_info_server',
                    parameters=[nav2_params],
                ),
            ],
        )
        on_start_actions.append(TimerAction(period=2.0, actions=[keepout_nodes]))
        # Dedicated lifecycle manager brings the two filter servers up (autostart)
        # independently of nav_central's nav2 startup.
        keepout_lifecycle = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output=log_output,
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
            }],
        )

    # Delay node loading until container process is running
    delayed_nav2_load = RegisterEventHandler(
        condition=IfCondition(nav2_activate),
        event_handler=OnProcessStart(
            target_action=nav2_container,
            on_start=on_start_actions,
        )
    )

    actions = [nav2_container, delayed_nav2_load]
    if keepout_lifecycle is not None:
        actions.append(keepout_lifecycle)

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
