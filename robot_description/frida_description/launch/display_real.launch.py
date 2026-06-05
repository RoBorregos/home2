#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Process URDF xacro file
    urdf_file = PathJoinSubstitution([FindPackageShare('frida_description'), 'urdf', 'TMR2025', 'FRIDA_Real.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_file])

    # robot_state_publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    ))

    # joint_state_publisher_gui
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    ))

    # rviz2
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    ))

    return ld
