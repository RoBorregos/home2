
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','3.1416','0','base_link','laser'],
        ),
        Node(
            package='nav_main',
            executable='ignore_laser',
            parameters = [{'ignore_array': '-138.4, -120.4, -86.9, -78.4, -39.8, -32.3, -22.3, -4.8, 33.3, 39.8, 77.9, 86.9, 122.4, 139.9'}],
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate': 460800, 
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Standard'}],
            output='screen',
            remappings=[
            ('/scan', '/scan_input')],
            ),

    ])
