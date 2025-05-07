from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav_main')
    params=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
    param_substitutions = {
        'use_sim_time': 'false',
        'autostart': 'true'}
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    nav2_nodes = [
        ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings ),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings),
            # ComposableNode(
            #     package='nav2_costmap_2d',
            #     plugin='nav2_costmap_2d::CostmapClient',
            #     name='local_costmap',
            #     parameters=[configured_params],
            # ),
        ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother',
                    #'local_costmap'
                ]
            }],
        ),
        
    ]

    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nav2_nodes,
        parameters=[configured_params],
        output='screen',
    )

    return LaunchDescription([container])
