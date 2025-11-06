import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from speech.speech_api_utils import SpeechApiUtils

from frida_constants import ModuleNames, parse_ros_config


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

    oww_config = parse_ros_config(
        os.path.join(get_package_share_directory("speech"), "config", "kws_oww.yaml"),
        [ModuleNames.HRI.value],
    )["kws_oww"]["ros__parameters"]

    aec_config = parse_ros_config(
        os.path.join(get_package_share_directory("speech"), "config", "aec.yaml"),
        [ModuleNames.HRI.value],
    )["aec_node"]["ros__parameters"]

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
            executable="aec_node.py",
            name="aec",
            output="screen",
            emulate_tty=True,
            parameters=[aec_config],
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
        Node(
            package="speech",
            executable="kws_oww.py",
            name="kws_oww",
            output="screen",
            emulate_tty=True,
            parameters=[oww_config],
        ),
    ]

    if SpeechApiUtils.respeaker_available():
        print("ReSpeaker detected - adding respeaker node to launch")
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
    else:
        print("ReSpeaker not detected - skipping respeaker node")

    return LaunchDescription(nodes)
