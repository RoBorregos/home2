import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def launch_setup(context, *args, **kwargs):

    laser_launch = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_rear',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyOmniLidar1',
            'serial_baudrate': 460800,
            'frame_id': 'lidar_rear',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'ignore_array': '-5, 95.0',
            'min_range': 0.0,
        }],
        respawn=True,
        respawn_delay=2.0,
        remappings=[
            ('/scan', '/scan_rear'),
        ]
    )

    
    laser2_launch = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_front',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyOmniLidar2',
            'serial_baudrate': 460800,
            'frame_id': 'lidar_front',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'ignore_array': '-5, 95.0',
            'min_range': 0.0,
        }],
        respawn=True,
        respawn_delay=2.0,
        remappings=[
            ('/scan', '/scan_front'),
        ]
    )

    merger = Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            output='screen',
            parameters=[{
                'destination_frame': 'base_link',
                'scan_destination_topic': '/scan',
                'cloud_destination_topic': '/merged_cloud',
                'laserscan_topics': '/scan_rear /scan_front',
                
                # Full 360-degree coverage in radians (-Pi to Pi)
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                
                # --- Slamtec RPLIDAR C1 Specific Parameters ---
                
                # 0.72 degrees converted to radians
                'angle_increment': 0.012566,
                
                # 10Hz typical scanning frequency (1/10)
                'scan_time': 0.1,
                
                # C1 physical range capabilities in meters
                'range_min': 0.05,
                'range_max': 12.0
            }]
        )
    return_launch = [
        laser_launch,
        laser2_launch,
        merger
    ]
    return return_launch

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
