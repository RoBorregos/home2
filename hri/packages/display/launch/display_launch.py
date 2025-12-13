from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros


def generate_launch_description():
    rosbridge_node = launch_ros.actions.Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_server",
        output="screen",
    )

    web_video_node = launch_ros.actions.Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
    )

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["npm", "run", "dev"],
                cwd=["/workspace/src/hri/packages/display/display"],
                output="screen",
            ),
            rosbridge_node,
            web_video_node,
        ]
    )
