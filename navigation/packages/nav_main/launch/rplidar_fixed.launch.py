
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import  DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Whether to use simulation time'
    )
    return LaunchDescription([
        declare_use_sim,
        Node(
            package='nav_main',
            executable='ignore_laser',
            parameters = [{'ignore_array': '-139.9, -122.4, -86.4, -78.4, -39.8, -34.3, 3.3, 21.8, 32.8, 38.3, 78.4, 86.9, 120.9, 138.4'}],
	    #parameters = [{'ignore_array': '-138.4, -120.4, -86.9, -78.4, -39.8, -32.3, -22.3, -4.8, 33.3, 39.8, 77.9, 86.9, 122.4, 139.9'}],
            condition=UnlessCondition(use_sim)
        ),
        Node(
            package='nav_main',
            executable='ignore_laser',
            #parameters=[{'ignore_array': '-176 ,-160, -129,-119, -85, -67, -56, -46, -15,5'}],
            condition=IfCondition(use_sim)
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate': 460800, 
                         'frame_id': 'laser',
                         'inverted': True,
                         'angle_compensate': True,
                         'scan_mode': 'Standard'}],
            output='screen',
            remappings=[
            ('/scan', '/scan_input')],
            condition=UnlessCondition(use_sim),
            ),

    ])
