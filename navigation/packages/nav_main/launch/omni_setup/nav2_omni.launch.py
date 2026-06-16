from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_file_route = get_package_share_directory('nav_main')
    nav2_params_file = os.path.join(pkg_file_route, 'config', 'omni_config', 'nav2_omni.yaml')
    nav2_activate = LaunchConfiguration('nav2', default='true')

    nav2_params = ParameterFile(
        LaunchConfiguration('nav2_config_file', default=nav2_params_file),
        allow_substs=True)

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

    return LaunchDescription([
        nav2_container,
        delayed_nav2_load,
    ])
