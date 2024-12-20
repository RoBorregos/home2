import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'output_frame': 'odom_filtered',
                    'frequency': 20.0,
                    'sensor_timeout': 0.2,
                    'two_d_mode': True,
                    'publish_tf': True,
                    'map_frame': 'map',
                    'base_link_frame': 'base_footprint',
                    'world_frame': 'odom',
                    'odom0': 'odom',
                    'imu0': 'imu',
                    #  x, y, z 
                    # roll, pitch, yaw 
                    #  vx, vy, vz 
                    # vroll, vpitch, vyaw
                    # ax, ay, az
                    'odom0_config': [True, True, False, False,  False,  True, True, False, False, False,  False,  True, False,  False,  False],
                    'imu0_config': [False, False, False, False,  False,  True, False, False, False, False,  False,  True, False,  False,  False],

                 },
                
            ],
        ),
    ])