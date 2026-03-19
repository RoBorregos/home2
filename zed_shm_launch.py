"""Wrapper launch that disables parameter_events for ALL nodes (including
robot_state_publisher) before loading the ZED camera launch file.

This prevents iceoryx TOO_MANY_CHUNKS_HELD_IN_PARALLEL errors that block
TF delivery when SHM is enabled.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():
    zed_launch = os.path.join(
        '/opt/zed_ws/install/zed_wrapper/share/zed_wrapper/launch',
        'zed_camera.launch.py',
    )

    camera_model = os.environ.get('ZED_CAMERA_MODEL', 'zed2')

    return LaunchDescription([
        # Globally disable parameter_events publisher for every node
        SetParameter(name='start_parameter_event_publisher', value=False),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                'camera_model': camera_model,
                'publish_tf': 'false',
                'ros_params_override_path': '/opt/zed_ws/zed_shm_override.yaml',
            }.items(),
        ),
    ])


if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
