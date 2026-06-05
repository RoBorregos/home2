 #!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    frida_description_path = FindPackageShare('frida_description')
    urdf_file = PathJoinSubstitution([frida_description_path, 'urdf', 'omnibase', 'robot.xacro'])

    # Process the URDF with xacro
    robot_description_content = Command(['xacro ', urdf_file])

    # Publish static transform from world to the robot base footprint
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'home_base_base_footprint'],
        output='screen'
    ))

    # Launch robot_state_publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    ))

    # Launch joint_state_publisher_gui
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    ))

    # Launch RViz
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    ))

    return ld