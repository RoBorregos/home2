
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_main',
            executable='ignore_laser',
            parameters = [{'ignore_array': '-176 ,-166, -129,-119, -85, -67, -56, -46, -10,5'}]
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
            ('/scan', '/scan_input')]
            ),

    ])
