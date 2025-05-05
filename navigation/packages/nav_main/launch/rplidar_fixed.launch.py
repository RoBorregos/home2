
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
            parameters = [{'ignore_array': '-176 ,-166, -129,-119, -85, -67, -56, -46, -10,5, 20,80, 126,157'}],
            condition=UnlessCondition(use_sim)
        ),
        Node(
            package='nav_main',
            executable='ignore_laser',
            parameters=[{'ignore_array': '-176 ,-160, -129,-119, -85, -67, -56, -46, -15,5'}],
            condition=IfCondition(use_sim)
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate': 115200, 
                         'frame_id': 'laser',
                         'inverted': True, 
                         'angle_compensate': True}],
            output='screen',
            remappings=[
            ('/scan', '/scan_input')],
            condition=UnlessCondition(use_sim),
            ),

    ])
