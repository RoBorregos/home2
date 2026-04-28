import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='contact_graspnet_ros',
            executable='contact_graspnet_node',
            name='contact_graspnet_node',
            output='screen',
            parameters=[{
                'ckpt_dir': 'checkpoints/contact_graspnet',
                'forward_passes': 1,
                'z_range': [0.2, 1.8],
            }]
        )
    ])
