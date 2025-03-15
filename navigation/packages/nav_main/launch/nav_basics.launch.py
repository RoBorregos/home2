import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    publish_urdf = LaunchConfiguration('publish_tf')
    enable_pointcloud_to_laserscan = LaunchConfiguration('enable_pointcloud_to_laserscan')
    
    # Declare the launch argument
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',  # LaunchConfiguration values are strings, so use 'true'/'false'
        description='Whether to publish URDF'
    )
    
    declare_enable_pointcloud_to_laserscan = DeclareLaunchArgument(
        'enable_pointcloud_to_laserscan',
        default_value='false',  # Default value for activation is 'false'
        description='Whether to enable the pointcloud to laserscan node'
    )

    dashgo_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dashgo_driver"),
                    "launch",
                    "dashgo_driver.launch.py",
                ]
            )
        ))
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dashgo_driver"),
                    "launch",
                    "ekf.launch.py",
                ]
            )
        ))
    
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("urdf_launch"),
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
        launch_arguments={
            'urdf_package': 'frida_description',
            'urdf_package_path': PathJoinSubstitution(['urdf','FRIDA_Real.urdf.xacro'])
        }.items(),
        condition=IfCondition(publish_urdf),  # Condition to include this launch file only if publish_tf is true
    )

    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav_main"),
                    "launch",
                    "rplidar_fixed.launch.py",
                ]
            )
        ))
    
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        parameters=[],
        remappings=[
            ('cloud_in', '/depth/color/points')  # Remap input topic
        ],
        arguments=['--ros-args', '-p', 'target_frame:=base_link'],  
        condition=IfCondition(enable_pointcloud_to_laserscan)
    )
    
    joint_state = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(publish_urdf),  # Only launch joint_state if publish_tf is true
    )

    return LaunchDescription([
        declare_enable_pointcloud_to_laserscan, 
        declare_publish_tf, 
        pointcloud_to_laserscan_node,
        dashgo_driver,
        ekf_launch,
        robot_description_launch,
        joint_state,
        laser_launch
    ])
