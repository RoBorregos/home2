import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_function(context, *args, **kwargs):
    pkg_file_route = get_package_share_directory('nav_main')
    rtab_params_file = os.path.join(pkg_file_route,'config', 'rtabmap', 'rtabmap_localization_config.yaml')
    nav2_params_file = os.path.join(pkg_file_route,'config', 'nav2_standard.yaml')
    rtabmap_map_name = LaunchConfiguration('map_name', default=os.getenv('MAP_NAME'))
    rtab_params = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    nav2_params = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    localization = LaunchConfiguration('localization', default='true')
    nav2_activate = LaunchConfiguration('nav2', default='true')
    use_sim = LaunchConfiguration('use_sim', default='false')

    # Adaptive Goal Parameters
    approach_min_dist = LaunchConfiguration('approach_min_dist', default='0.5')
    approach_max_dist = LaunchConfiguration('approach_max_dist', default='1.2')

    areas_map_name = context.perform_substitution(rtabmap_map_name).replace('.db', '')

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
            }],
    )

    nav_ui_node = Node(
        package='map_context',
        executable='nav_ui.py',
        name='nav_ui',
        output='screen',
        parameters=[{'map_name': rtabmap_map_name}],
    )

    nav_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "nav_basics.launch.py"])
        ),
    )

    rtabmapnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "rtabnav2.launch.py"])
        ),
        launch_arguments={'localization': localization, 'rtab_config_file': rtab_params, 'nav2_config_file': nav2_params, 'nav2': nav2_activate, 'map_name': rtabmap_map_name}.items(),
    )

    adaptive_goal_node = Node(
        package='nav_main',
        executable='adaptive_goal_publisher.py',
        name='adaptive_goal_publisher',
        output='screen',
        parameters=[{
            'approach_min_dist': approach_min_dist,
            'approach_max_dist': approach_max_dist,
            'use_sim_time': use_sim
        }]
    )

    return [
        nav_central_node,
        nav_ui_node,
        nav_basics,
        rtabmapnav,
        adaptive_goal_node
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_function)])
