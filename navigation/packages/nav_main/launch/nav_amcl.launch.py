import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lifecycle_nodes = ['map_server', 'amcl']

    publish_urdf = LaunchConfiguration('publish_tf')
    use_sim = LaunchConfiguration('use_sim')
    use_dualshock = LaunchConfiguration('use_dualshock')
    map_route = LaunchConfiguration('map')

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

    declare_map_route = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('nav_main'), 'maps', 'Lab14marzo.yaml'),
        description='Path to the map file'
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

    map_server = Node(
         package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_route,
                         'use_sim_time': use_sim}],
    )
    amcl_server = Node(
         package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
    )
    
    lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])
    
    return LaunchDescription([
        declare_publish_tf,
        nav_basics,
        declare_map_route,
        map_server,
        amcl_server,
        lifecycle_node,
        declare_dualshock,
        declare_use_sim

    ])
