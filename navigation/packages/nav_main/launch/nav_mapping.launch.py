import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav_main_package = get_package_share_directory('nav_main')
    params_file = os.path.join(nav_main_package, 'config', 'mapper_params_online_sync.yaml')
    
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
        default_value='true',  # LaunchConfiguration values are strings, so use 'true'/'false'
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
        #launch_arguments={'publish_tf': publish_urdf }.items()
        launch_arguments={'publish_tf': publish_urdf, 'use_sim': use_sim, 'use_dualshock' : use_dualshock}.items()
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
            launch_arguments={'params_file': params_file, 'use_sim_time': use_sim}.items()
        )
    
    return LaunchDescription([
        declare_publish_tf,
        declare_use_sim,
        declare_dualshock,
        nav_basics,
        slam_toolbox,
    ])
