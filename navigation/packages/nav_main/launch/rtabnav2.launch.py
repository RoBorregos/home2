from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_file_route = get_package_share_directory('nav_main')
    mapping_params_file = os.path.join(pkg_file_route, 'config', 'rtabmap', 'rtabmap_mapping_config.yaml')
    localization_params_file = os.path.join(pkg_file_route, 'config', 'rtabmap', 'rtabmap_localization_config.yaml')
    nav2_params_file = os.path.join(pkg_file_route, 'config', 'nav2_standard.yaml')
    localization = LaunchConfiguration('localization', default='false')
    nav2_activate = LaunchConfiguration('nav2', default='true')
    load_rtab = LaunchConfiguration('load_rtab', default='false')
    rtabmap_map_name = LaunchConfiguration('map_name', default='rtabmap_map.db')
    use_dualshock = LaunchConfiguration('use_dualshock', default='true')

    nav2_params = ParameterFile(
        LaunchConfiguration('nav2_config_file', default=nav2_params_file),
        allow_substs=True)

    map_substitution = {'database_path': ['/workspace/src/navigation/rtabmapdbs/', rtabmap_map_name]}

    mapping_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('rtab_mapping_config', default=mapping_params_file),
            root_key='',
            param_rewrites=map_substitution,
            convert_types=True),
        allow_substs=True)

    localization_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('rtab_localization_config', default=localization_params_file),
            root_key='',
            param_rewrites=map_substitution,
            convert_types=True),
        allow_substs=True)

    from launch.conditions import UnlessCondition
    from frida_constants.navigation_constants import CAMERA_RGB_TOPIC, CAMERA_INFO_TOPIC, CAMERA_DEPTH_TOPIC

    log_output = 'own_log' if os.getenv('NAV_QUIET') == '1' else 'screen'
    topic_name = LaunchConfiguration('topic_name', default='/cmd_vel')

    sync_remapping = [
        ('rgb/image', CAMERA_RGB_TOPIC),
        ('rgb/camera_info', CAMERA_INFO_TOPIC),
        ('depth/image', CAMERA_DEPTH_TOPIC)]

    # Container 1: RTAB-Map (heavy SLAM + image sync)
    rtabmap_container = ComposableNodeContainer(
        name='rtabmap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output=log_output,
        respawn=True,
        respawn_delay=2.0,
        composable_node_descriptions=[
            ComposableNode(
                condition=IfCondition(PythonExpression(["'", load_rtab, "' == 'true' and '", localization, "' == 'true'"])),
                package='rtabmap_slam',
                plugin='rtabmap_slam::CoreWrapper',
                name='rtabmap',
                parameters=[localization_params],
            ),
            ComposableNode(
                condition=IfCondition(PythonExpression(["'", load_rtab, "' == 'true' and '", localization, "' != 'true'"])),
                package='rtabmap_slam',
                plugin='rtabmap_slam::CoreWrapper',
                name='rtabmap',
                parameters=[mapping_params],
            ),
            ComposableNode(
                condition=IfCondition(load_rtab),
                package='rtabmap_sync',
                plugin='rtabmap_sync::RGBDSync',
                name='rgbd_sync',
                parameters=[mapping_params],
                remappings=sync_remapping
            ),
        ],
    )

    # Joy + DualShock controller loaded into rtabmap container
    dualshock_nodes = LoadComposableNodes(
        condition=IfCondition(use_dualshock),
        target_container='rtabmap_container',
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy',
                namespace='',
            ),
            ComposableNode(
                package='nav_main',
                plugin='nav_main::ControllerDualshock',
                name='controller_dualshock',
                namespace='',
                parameters=[{
                    'linear_speed': 0.2,
                    'rotation_speed': 0.1,
                    'deadzone': 0.4,
                }],
                remappings=[
                    ('cmd_vel', topic_name)
                ],
            ),
        ],
    )

    # Delay dualshock loading until rtabmap container is running
    delayed_dualshock_load = RegisterEventHandler(
        condition=IfCondition(use_dualshock),
        event_handler=OnProcessStart(
            target_action=rtabmap_container,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[dualshock_nodes]
                )
            ]
        )
    )

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

    return LaunchDescription([
        rtabmap_container,
        nav2_container,
        delayed_nav2_load,
        delayed_dualshock_load,
    ])
