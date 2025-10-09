import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():
    pkg_file_route = get_package_share_directory('nav_main')
    rtab_params_file = os.path.join(pkg_file_route,'config', 'rtabmap', 'rtabmap_default_config.yaml')
    nav2_params_file = os.path.join(pkg_file_route,'config', 'nav2_standard.yaml')
    rtabmap_map_name = LaunchConfiguration('map_name', default='rtabmap_robocuptesting.db')
    rtab_params = LaunchConfiguration('rtab_config_file', default=rtab_params_file)
    nav2_params = LaunchConfiguration('nav2_config_file', default=nav2_params_file)
    localization = LaunchConfiguration('localization', default='true')
    nav2_activate = LaunchConfiguration('nav2', default='true')
    use_sim = LaunchConfiguration('use_sim', default='false')

    nav_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "nav_basics.launch.py",
                ]
            )),
        launch_arguments={'use_sim': use_sim}.items()
        )

    rtabmapnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "rtabnav2.launch.py",
                ]
            )),
        launch_arguments={'localization': localization, 'rtab_config_file': rtab_params, 'nav2_config_file': nav2_params, 'nav2': nav2_activate, 'map_name': rtabmap_map_name}.items(),
        )
    
    return LaunchDescription([
        nav_basics,
        rtabmapnav,
        
    ])