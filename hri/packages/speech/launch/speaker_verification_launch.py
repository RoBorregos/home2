"""Speaker verification (issue #1459), standalone.

    ros2 launch speech speaker_verification_launch.py                     # node only
    ros2 launch speech speaker_verification_launch.py with_capture:=true  # + mic capture
    ros2 launch speech speaker_verification_launch.py with_capture:=true with_legacy_vad:=true

``with_capture`` starts audio_capturer (needed when the full HRI stack is not
running); ``with_legacy_vad`` also starts noise_cancellation + voice_detection so
the monitor can compare the old VAD side by side. Extra node parameters can be
overridden with ``params:=/path/to/override.yaml``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from frida_constants import ModuleNames, parse_ros_config

LOG_LEVEL = os.environ.get("HRI_LOG_LEVEL", "info")


def _config(file: str, node: str) -> dict:
    return parse_ros_config(
        os.path.join(get_package_share_directory("speech"), "config", file),
        [ModuleNames.HRI.value],
    )[node]["ros__parameters"]


def _truthy(value: str) -> bool:
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    with_capture = _truthy(context.launch_configurations["with_capture"])
    with_legacy = _truthy(context.launch_configurations["with_legacy_vad"])
    override = context.launch_configurations["params"]

    params = [_config("speaker_verification.yaml", "speaker_verification")]
    if override:
        params.append(override)

    nodes = [
        Node(
            package="speech",
            executable="speaker_verification.py",
            name="speaker_verification",
            ros_arguments=["--log-level", f"speaker_verification:={LOG_LEVEL}"],
            output="screen",
            emulate_tty=True,
            parameters=params,
        )
    ]
    if with_capture:
        nodes.append(
            Node(
                package="speech",
                executable="audio_capturer.py",
                name="audio_capturer",
                ros_arguments=["--log-level", f"audio_capturer:={LOG_LEVEL}"],
                output="screen",
                emulate_tty=True,
                parameters=[_config("microphone.yaml", "audio_capturer")],
            )
        )
    if with_legacy:
        nodes += [
            Node(
                package="speech",
                executable="noise_cancellation.py",
                name="noise_cancellation",
                ros_arguments=["--log-level", f"noise_cancellation:={LOG_LEVEL}"],
                output="screen",
                emulate_tty=True,
                parameters=[_config("noise_cancellation.yaml", "noise_cancellation")],
            ),
            Node(
                package="speech",
                executable="voice_detection.py",
                name="voice_detection",
                ros_arguments=["--log-level", f"voice_detection:={LOG_LEVEL}"],
                output="screen",
                emulate_tty=True,
                parameters=[_config("voice_detection.yaml", "voice_detection")],
            ),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("with_capture", default_value="false"),
            DeclareLaunchArgument("with_legacy_vad", default_value="false"),
            DeclareLaunchArgument("params", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
