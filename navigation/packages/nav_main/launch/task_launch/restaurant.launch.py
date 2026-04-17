import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_function(context, *args, **kwargs):
    pkg_file_route = get_package_share_directory('nav_main')

    # Mapping config — RTAB-Map builds a fresh map (delete_db_on_start=true)
    rtab_params_file = os.path.join(
        pkg_file_route, 'config', 'rtabmap', 'rtabmap_mapping_config.yaml'
    )
    nav2_params_file = os.path.join(pkg_file_route, 'config', 'nav2_restaurant.yaml')

    rtab_params = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    nav2_params = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    rtabmap_map_name = LaunchConfiguration('map_name', default='restaurant_session.db')

    # Nav Central — hybrid mode: mapping=True + use_nav2=True
    # SLAM builds the map incrementally while Nav2 navigates within it
    nav_central_node = Node(
        package='nav_main',
        executable='nav_central.py',
        name='nav_central',
        namespace='',
        output='screen',
        parameters=[{
            'mapping': True,
            'localization': False,
            'use_nav2': True,
            'map_name': rtabmap_map_name,
            'rtab_mapping_config': rtab_params,
            'rtab_localization_config': rtab_params,
        }],
    )

    nav_ui_node = Node(
        package='map_context',
        executable='nav_ui.py',
        name='nav_ui',
        output='screen',
        parameters=[{
            'mode': 'mapping',
            'map_name': rtabmap_map_name,
        }],
    )

    nav_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "nav_basics.launch.py"])
        ),
    )

    # RTAB-Map + Nav2 stack: mapping mode with Nav2 enabled
    rtabmapnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav_main"), "launch", "rtabnav2.launch.py"])
        ),
        launch_arguments={
            'localization': 'false',
            'rtab_config_file': rtab_params,
            'nav2_config_file': nav2_params,
            'nav2': 'true',
            'map_name': rtabmap_map_name,
        }.items(),
    )

    # Adaptive goal publisher — adjusts goals that land inside obstacles
    # (e.g., table centers) using radial sampling / BFS
    adaptive_goal_node = Node(
        package='nav_main',
        executable='adaptive_goal_publisher.py',
        name='adaptive_goal_publisher',
        output='screen',
    )

    return [
        nav_central_node,
        nav_ui_node,
        nav_basics,
        rtabmapnav,
        adaptive_goal_node,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_function)])
