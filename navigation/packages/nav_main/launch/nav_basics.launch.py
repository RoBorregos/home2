import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    publish_urdf = LaunchConfiguration('publish_tf', default=True)

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
    
    joint_state = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    if publish_urdf:
        return LaunchDescription([
            dashgo_driver,
            ekf_launch,
            robot_description_launch,
            joint_state,
            laser_launch
        ])
    else:
        return LaunchDescription([
            dashgo_driver,
            ekf_launch,
            laser_launch
        ])