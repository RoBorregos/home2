import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition

def launch_function(context, *args, **kwargs):
    pkg_file_route = get_package_share_directory('nav_main')
    rtab_params_file = os.path.join(pkg_file_route,'config', 'rtabmap', 'rtabmap_default_config.yaml')
    nav2_params_file = os.path.join(pkg_file_route,'config', 'nav2_standard.yaml')
    rtabmap_map_name = LaunchConfiguration('map_name', default=os.getenv('MAP_NAME'))
    rtab_params = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    nav2_params = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    localization = LaunchConfiguration('localization', default='true')
    nav2_activate = LaunchConfiguration('nav2', default='true')
    use_sim = LaunchConfiguration('use_sim', default='false')

    nav_manager_node = LifecycleNode(
        package='nav_main',
        executable='nav_lifecycle_manager.py',
        name='nav_lifecycle_manager',
        namespace='',
        output='screen',
        parameters=[{'managed_nodes': ['map_service'],
                      'start_parameter_event_publisher': False}],
    )


    nav_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "nav_basics.launch.py"])
        ),
        launch_arguments={'use_sim': use_sim}.items()
    )

    rtabmapnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "rtabnav2.launch.py"])
        ),
        launch_arguments={'localization': localization, 'rtab_config_file': rtab_params, 'nav2_config_file': nav2_params, 'nav2': nav2_activate, 'map_name': rtabmap_map_name}.items(),
    )

    map_name_str = rtabmap_map_name.perform(context)
    map_context_node = LifecycleNode(
        package='map_context',
        executable='map_service',
        name='map_service',
        namespace='',
        parameters=[{'map_name': map_name_str[:-3],
                      'start_parameter_event_publisher': False}],
        output='screen',
    )

    from launch_ros.event_handlers import OnStateTransition
    
    wait_for_activation = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=nav_manager_node,
            goal_state='active',
            entities=[
                LogInfo(msg="[NavComposition] Dependencias externas listas"),
                nav_basics,
                rtabmapnav,
            ]
        )
    )

    on_deactivation = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=nav_manager_node,
            goal_state='deactivating',
            entities=[
                LogInfo(msg="[NavComposition] Manager desactivándose - Perdiendo dependencias"),
            ]
        )
    )

    return [
        nav_manager_node,
        map_context_node,
        wait_for_activation,
        on_deactivation
    ]
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_function)])
