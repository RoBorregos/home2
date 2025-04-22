import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition

def launch_setup(context, *args, **kwargs):
    lifecycle_nodes = ['map_server', 'amcl']
    nav_dir = get_package_share_directory('nav_main')
    use_sim = LaunchConfiguration('use_sim', default='false')
    localization = LaunchConfiguration('localization', default='true')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz', default='false')
    default_value=os.path.join(nav_dir, 'config', 'storing_groceries.yaml'),
    params_file = LaunchConfiguration('params_file', default=default_value)
    use_amcl = LaunchConfiguration('use_amcl', default='false')
    map_route = LaunchConfiguration('map', default_value=os.path.join(get_package_share_directory('nav_main'), 'maps', 'Lab14marzo.yaml'),)
    
    
    nav_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "nav_basics.launch.py",
                ]
            )),
        launch_arguments={'use_sim': use_sim,}.items()
        )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "nav2.launch.py",
                ]
            )),
        launch_arguments={'use_sim_time': use_sim, 'params_file': params_file}.items()
        )
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "rtabmap_slam.launch.py",
                ]
            )),
        launch_arguments={'use_sim_time': use_sim, 'localization': localization, 'rtabmap_viz': rtabmap_viz}.items(),
        condition=UnlessCondition(use_amcl)
        )
    
    map_server = Node(
         package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_route,
                         'use_sim_time': use_sim}],
        condition=IfCondition(use_amcl)
    )
    amcl_server = Node(
         package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
            condition=IfCondition(use_amcl)
    )
    
    lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}],
                        condition=IfCondition(use_amcl)
            )
    
    return [
        nav_basics,
        rtabmap,
        nav2_launch,
        map_server,
        amcl_server,
        lifecycle_node,
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])