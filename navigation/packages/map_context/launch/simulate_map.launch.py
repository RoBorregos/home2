from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    pkg_file_route = get_package_share_directory('map_context')
    map_path = os.path.join(pkg_file_route,'maps', 'Lab01.yaml')
    rviz2_path = os.path.join(pkg_file_route,'config', 'rviz','simulation_map.rviz')
    rosbag_path = os.path.join(pkg_file_route,'config', 'rosbag','basic_vision')
    default_map = LaunchConfiguration('map_path', default = map_path)
    rviz2_config = LaunchConfiguration('rviz2_config', default = rviz2_path)
    rosbag_config = LaunchConfiguration('rosbag_config', default = rosbag_path)

    container = [
        Node(
            package='map_context',
            executable='simulate_position.py',
            name='simulateposition',
            parameters=[{
                "use_sim_time": True 
                }]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                "yaml_filename" :  default_map,
                "use_sim_time": True 
                }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            emulate_tty=True,
            parameters=[{
                "use_sim_time": True 
                }],
            arguments=[
                '-d', rviz2_config       
                ]),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '--clock','--loop', rosbag_config ],
            output='screen'
        )]

    return LaunchDescription(container)
