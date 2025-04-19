 #!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'frida_description',
            'urdf_package_path': PathJoinSubstitution(['urdf','RoboCup2024','FRIDA.urdf.xacro'])}.items()
    ))

    ld.add_action(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             ))

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    ld.add_action(Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'Frida',
            '-entity', 'UF_ROBOT',
            '-x', '-0.2',
            '-y', '-0.5',
            '-z', '0.25',
        ],
        parameters=[{'use_sim_time': True}],
    ))
    return ld