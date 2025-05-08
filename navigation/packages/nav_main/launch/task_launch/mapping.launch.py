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
    rviz_config_dir = os.path.join(get_package_share_directory('nav_main'), 'rviz_configs', 'mapping.rviz')
    nav_main_package = get_package_share_directory('nav_main')
    params_file = os.path.join(nav_main_package, 'config', 'map_sync_slam.yaml')
    use_sim = LaunchConfiguration('use_sim', default='false')
    localization = LaunchConfiguration('localization', default='false')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz', default='false')
    use_3d = LaunchConfiguration('use_3d', default='false')
    show_rviz = LaunchConfiguration('show_rviz', default='true')

    use_slamtoolbox = LaunchConfiguration('use_slam', default='false')

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

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_sync_launch.py",
                ]
            )),
            launch_arguments={'params_file': params_file, 'use_sim_time': use_sim}.items(),
            condition=IfCondition(use_slamtoolbox)
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
        launch_arguments={'use_sim_time': use_sim, 'localization': localization, 'rtabmap_viz': rtabmap_viz, '3d_grid': use_3d}.items(),
        condition=UnlessCondition(use_slamtoolbox)
        )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(show_rviz)
    )
    
    return LaunchDescription([
        nav_basics,
        slam_toolbox,
        rtabmap,
        rviz_node

    ])
