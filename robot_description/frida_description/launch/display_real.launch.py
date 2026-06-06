#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    # Process URDF xacro file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('frida_description'),
        'urdf', 'TMR2025', 'FRIDA_Real.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' robot_ip:=0.0.0.0',
            ' prefix:=',
            ' ros2_control_plugin:=fake_components/GenericSystem',
        ]),
        value_type=str
    )

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='frida_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='frida_joint_state_publisher_gui',
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    ))

    return ld