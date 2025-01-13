import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mic_config = os.path.join(
        get_package_share_directory("speech"), "config", "microphone.yaml"
    )

    speaker_config = os.path.join(
        get_package_share_directory("speech"), "config", "speaker.yaml"
    )
    # respeaker_config = os.path.join(
    #     get_package_share_directory("speech"), "config", "respeaker.yaml"
    # )

    return LaunchDescription(
        [
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
                executable="say.py",
                name="say",
                output="screen",
                emulate_tty=True,
                parameters=[speaker_config],
            ),
            # Node(
            #     package="speech",
            #     executable="respeaker.py",
            #     name="respeaker",
            #     output="screen",
            #     emulate_tty=True,
            #     parameters=[respeaker_config],
            # ),
        ]
    )
