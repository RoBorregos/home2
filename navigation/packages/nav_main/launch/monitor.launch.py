from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_main',
            executable='node_monitor.py',
            name='node_monitor',
            parameters=[{
                'nodes_to_monitor': [
                    'amcl', 
                    'bt_navigator', 
                    'controller_server', 
                    'planner_server', 
                    'map_server',
                    'dashgo_driver',
                    'rplidar_node'
                ]
            }]
        )
    ])
