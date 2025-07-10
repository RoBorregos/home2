from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    pkg_file_route = get_package_share_directory('nav_main')
    nav2_params_file = os.path.join(pkg_file_route,'config', 'nav2_standard.yaml')
    nav2_params_ = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    docking = LaunchConfiguration('use_docking', default='false')

    nav2_params = ParameterFile(nav2_params_, allow_substs=True)

    life_cyle_params = {
        'use_sim_time': False,
        'autostart': True,
        'node_names': [
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'velocity_smoother',
        ]
    }
    
    nav2_nodes = [
        ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[nav2_params]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[nav2_params]),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[nav2_params]),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[nav2_params]),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[nav2_params]),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[nav2_params],
                ),
            ComposableNode(
                package='opennav_docking',
                plugin='opennav_docking::DockingServer',
                name='docking_server',
                parameters=[nav2_params],
                condition=IfCondition(docking),
                ),
            
        ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            parameters=[life_cyle_params],
        ),

    ]
    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=nav2_nodes,
        parameters=[nav2_params],
        output='screen',
    )

    return LaunchDescription([container])
