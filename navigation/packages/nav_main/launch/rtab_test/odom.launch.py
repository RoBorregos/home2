
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    remappings=[
          ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
          ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
          ('depth/image', '/zed/zed_node/depth/depth_registered')]

    return_list = [
        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':False, 'use_sim_time':False , 'qos': 1, 'qos_camera_info': 1}],
            remappings=remappings),
    ]
    return return_list
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])