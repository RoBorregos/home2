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
from launch.actions import SetEnvironmentVariable, ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'frida_description'   

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'frida_description',
            'use_sim_time': 'true',
            'jsp_gui': 'false',
            'urdf_package_path': PathJoinSubstitution(['urdf','FRIDA.urdf.xacro'])}.items(),
            
    ))
    
    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", 
                    "-entity", "UF_ROBOT",
                    "-x", '0.0',
                    "-y", '0.0',
                    "-z", '0.05',
                    "-Y", '0.0']
    ))
    ld.add_action(ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',     
        '-s', 'libgazebo_ros_init.so'], output='screen',
        ))
    
    return ld