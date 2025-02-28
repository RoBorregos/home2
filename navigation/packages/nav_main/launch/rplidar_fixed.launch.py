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
        Node(
            package='nav_main',
            executable='ignore_laser',
            parameters = [{'ignore_array': '-176 ,-166, -129,-119, -85, -67, -56, -46, -10,5'}]
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB1', 
                         'serial_baudrate': 115200, 
                         'frame_id': 'laser',
                         'inverted': True, 
                         'angle_compensate': True}],
            output='screen',
            remappings=[
            ('/scan', '/scan_input')]
            ),

    ])

#<param name="ignore_array" type="string" value="-176 ,-166, -129,-119, -85, -67, -56, -46, -10,5" />