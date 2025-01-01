 #!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    
    ld = LaunchDescription()

    package_name = 'frida_description'
    # SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=get_package_share_directory(package_name))
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )


    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'frida_description',
            'urdf_package_path': PathJoinSubstitution(['urdf','FRIDA.urdf.xacro'])}.items()
    ))
    ld.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        arguments = ['-topic', 'robot_description',
                     '-name', 'my_bot',
                     '-z', '0.1' ],
        output="screen"
    )
    )
    ld.add_action(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', os.path.join( get_package_share_directory(package_name), 'worlds', 'empty.world')],
                                     'on_exit_shutdown': 'true'}.items(),
                    
             ))
    
    return ld