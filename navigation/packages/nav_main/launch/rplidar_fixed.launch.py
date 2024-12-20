# from launch_ros.substitutions import FindPackageShare
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, Node
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution, TextSubstitution

# def generate_launch_description():

#     return LaunchDescription([
#         # IncludeLaunchDescription(
#         #     PythonLaunchDescriptionSource([
#         #         PathJoinSubstitution([
#         #             FindPackageShare('sllidar_ros2'),
#         #             'launch',
#         #             'sllidar_a1.launch.py'
#         #         ])
#         #     ]),
#         #     launch_arguments={
#         #         'serial_port': '/dev/ttyUSB0',
#         #         'frame_id': 'laser',
#         #         'inverted':'true',
#         #     }.items()
#         # ),
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='laser_to_base_link',
#             parameters=[
#                 "0.1", "0.0", "0.2", "-1.57", "0.0", "0.0", "/base_link", "/laser", "40"
#             ],
#         ),
#     ])


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'sllidar_a1_launch.py'
                ])
            ]),
            launch_arguments={
                'serial_port': '/dev/ttyUSB1',
                'frame_id': 'laser',
                'inverted':'false',
            }.items()
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.1',
                          '--y', '0.0', 
                          '--z', '0.2', 
                          '--yaw', '-1.57', 
                          '--pitch', '0.0', 
                          '--roll', '0.0', 
                          '--frame-id', 'base_link', 
                          '--child-frame-id', 'laser']
        ),
    ])