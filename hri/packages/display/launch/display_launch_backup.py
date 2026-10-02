"""Legacy Next.js display (rosbridge + web_video_server + `npm run start`).

Kept alongside display_launch.py (the default PyQt UI) as a fallback; launch
with `ros2 launch display display_launch_backup.py` or `run.sh ... --backup`.
"""

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
                cmd=["npm", "run", "start"],
                cwd=["/workspace/src/hri/packages/display/display"],
                output="screen",
            ),
            rosbridge_node,
            web_video_node,
        ]
    )
