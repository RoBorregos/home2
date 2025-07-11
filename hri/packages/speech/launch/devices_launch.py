import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from frida_constants import ModuleNames, parse_ros_config
from frida_constants.hri_constants import USE_OWW, USE_RESPEAKER


def generate_launch_description():
    mic_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("speech"), "config", "microphone.yaml"
        ),
        [ModuleNames.HRI.value],
    )["audio_capturer"]["ros__parameters"]

    hear_streaming_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("speech"), "config", "hear_streaming.yaml"
        ),
        [ModuleNames.HRI.value],
    )["hear_streaming"]["ros__parameters"]

    speaker_config = parse_ros_config(
        os.path.join(get_package_share_directory("speech"), "config", "speaker.yaml"),
        [ModuleNames.HRI.value],
    )["say"]["ros__parameters"]

    respeaker_config = parse_ros_config(
        os.path.join(get_package_share_directory("speech"), "config", "respeaker.yaml"),
        [ModuleNames.HRI.value],
    )["respeaker"]["ros__parameters"]

    kws_config = parse_ros_config(
        os.path.join(get_package_share_directory("speech"), "config", "kws.yaml"),
        [ModuleNames.HRI.value],
    )["keyword_spotting"]["ros__parameters"]

    oww_config = parse_ros_config(
        os.path.join(get_package_share_directory("speech"), "config", "kws_oww.yaml"),
        [ModuleNames.HRI.value],
    )["kws_oww"]["ros__parameters"]

    nodes = [
        Node(
            package="speech",
            executable="audio_capturer.py",
            name="audio_capturer",
            output="screen",
            emulate_tty=True,
            parameters=[mic_config],
        ),
        Node(
            package="speech",
            executable="hear_streaming.py",
            name="hear",
            output="screen",
            emulate_tty=True,
            parameters=[hear_streaming_config],
        ),
        Node(
            package="speech",
            executable="say.py",
            name="say",
            output="screen",
            emulate_tty=True,
            parameters=[speaker_config],
        ),
    ]

    if USE_RESPEAKER:
        nodes.append(
            Node(
                package="speech",
                executable="respeaker.py",
                name="respeaker",
                output="screen",
                emulate_tty=True,
                parameters=[respeaker_config],
            )
        )

    if USE_OWW:
        nodes.append(
            Node(
                package="speech",
                executable="kws_oww.py",
                name="kws_oww",
                output="screen",
                emulate_tty=True,
                parameters=[oww_config],
            )
        )
    else:
        # Deprecated: use oww instead
        nodes.append(
            Node(
                package="speech",
                executable="kws.py",
                name="keyword_spotting",
                output="screen",
                emulate_tty=True,
                parameters=[kws_config],
            )
        )

    return LaunchDescription(nodes)
