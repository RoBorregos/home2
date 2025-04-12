from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_main',  
            executable='transform_target_node',  
            name='transform_target_node',
            output='screen'
        )
    ])
