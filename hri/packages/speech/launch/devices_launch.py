import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from frida_constants import ModuleNames, parse_ros_config

USE_RESPEAKER = False
USE_OWW = False


def generate_launch_description():
    mic_config = os.path.join(
        get_package_share_directory("speech"), "config", "microphone.yaml"
    )

    hear_config = parse_ros_config(
        os.path.join(get_package_share_directory("speech"), "config", "hear.yaml"),
        [ModuleNames.HRI.value],
    )["hear"]["ros__parameters"]

    speaker_config = parse_ros_config(
        os.path.join(get_package_share_directory("speech"), "config", "speaker.yaml"),
        [ModuleNames.HRI.value],
    )["say"]["ros__parameters"]

    respeaker_config = os.path.join(
        get_package_share_directory("speech"), "config", "respeaker.yaml"
    )

    kws_config = os.path.join(
        get_package_share_directory("speech"), "config", "kws.yaml"
    )
    useful_audio_config = os.path.join(
        get_package_share_directory("speech"), "config", "useful_audio.yaml"
    )

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
            executable="hear.py",
            name="hear",
            output="screen",
            emulate_tty=True,
            parameters=[hear_config],
        ),
        Node(
            package="speech",
            executable="say.py",
            name="say",
            output="screen",
            emulate_tty=True,
            parameters=[speaker_config],
        ),
        Node(
            package="speech",
            executable="useful_audio.py",
            name="useful_audio",
            output="screen",
            emulate_tty=True,
            parameters=[useful_audio_config],
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
            )
        )
    else:
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
