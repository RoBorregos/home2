import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("object_detector_2d"),
        "config",
        "parameters_zero_shot.yaml",
    )
    handler_launch_file = os.path.join(
        get_package_share_directory("object_detection_handler"),
        "launch",
        "objectDetectionHandler.launch.py",
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock (true) for the MuJoCo sim, wall time (false) for real robot.",
            ),
            Node(
                package="object_detector_2d",
                executable="zero_shot_object_detector_node.py",
                name="ObjectDetect2D",
                respawn=True,
                output="screen",
                emulate_tty=False,
                parameters=[config, {"use_sim_time": use_sim_time}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(handler_launch_file),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
