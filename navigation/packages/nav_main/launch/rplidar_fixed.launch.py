
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
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSBlidar2',
                         'serial_baudrate': 460800,
                         'frame_id': 'laser',
                         'inverted': True,
                         'angle_compensate': True,
                         'scan_mode': 'Standard',
                         'ignore_array': '-139.9, -122.4, -86.4, -78.4, -40.8, -30.3, 0, 21.8, 30.8, 40.3, 78.4, 93.9, 120.9, 138.4',
                         'min_range': 0.12}],
            output='screen',
            condition=UnlessCondition(use_sim),
            ),
    ])
