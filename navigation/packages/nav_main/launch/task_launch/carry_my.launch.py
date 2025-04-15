import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    publish_urdf = LaunchConfiguration('publish_tf')
    use_sim = LaunchConfiguration('use_sim')
    use_dualshock = LaunchConfiguration('use_dualshock')


    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',  # LaunchConfiguration values are strings, so use 'true'/'false'
        description='Whether to publish URDF'
    )

    declare_dualshock = DeclareLaunchArgument(
        'use_dualshock',
        default_value='false',  # LaunchConfiguration values are strings, so use 'true'/'false'
        description='Whether to use dualshock'
    )

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Whether to use simulation time'
    )


    nav_basics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "nav_basics.launch.py",
                ]
            )),
        launch_arguments={'publish_tf': publish_urdf, 'use_sim': use_sim, 'use_dualshock' : use_dualshock}.items()
        )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "navigation.launch.py",
                ]
            )),
        launch_arguments={'use_sim': use_sim}.items()
        )
    pose_transform = Node(
                package='nav_main',
                executable='transform_target.py',
                name='transform_target',
                output='screen',
                ),
    
    return LaunchDescription([
        declare_publish_tf,
        nav_basics,
        declare_dualshock,
        declare_use_sim,
        nav2_launch,
        pose_transform,
    ])
