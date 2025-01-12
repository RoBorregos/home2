import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('example_pkg'),
      'config',
      'example.yaml'
      )
    return LaunchDescription([
        Node(
            package='example_pkg',
            executable='example_pkg_node',
            name='example_pkg_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                config
            ],
        ),
    ])