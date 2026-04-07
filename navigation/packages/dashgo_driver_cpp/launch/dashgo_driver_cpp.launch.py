import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('dashgo_driver_cpp'),
      'config',
      'dashgo_params.yaml'
      )
    cmd_topic = LaunchConfiguration('cmd_topic', default='/cmd_vel')
    
    return LaunchDescription([
        Node(
            package='dashgo_driver_cpp',
            executable='dashgo_driver_cpp',
            name='DashgoDriverCpp',
            output='screen',
            emulate_tty=True,
            parameters=[config],
            remappings=[
                ('/cmd_vel', cmd_topic),
            ],
        ),
    ])
