import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('dashgo_driver'),
      'config',
      'dashgo_params.yaml'
      )
    return LaunchDescription([
        Node(
            package='dashgo_driver',
            executable='dashgo_driver2.py',
            name='DashgoDriver',
            output='screen',
            emulate_tty=True,
            parameters=[
                config
            ],
        ),
    ])