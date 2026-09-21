import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from speech.speech_api_utils import SpeechApiUtils

from frida_constants import ModuleNames, parse_ros_config

LOG_LEVEL = os.environ.get("HRI_LOG_LEVEL", "info")


def generate_launch_description():
    mic_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("speech"), "config", "microphone.yaml"
        ),
        [ModuleNames.HRI.value],
    )["audio_capturer"]["ros__parameters"]

    noise_cancellation_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("speech"), "config", "noise_cancellation.yaml"
        ),
        [ModuleNames.HRI.value],
    )["noise_cancellation"]["ros__parameters"]

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

    voice_detection_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("speech"), "config", "voice_detection.yaml"
        ),
        [ModuleNames.HRI.value],
    )["voice_detection"]["ros__parameters"]

    doorbell_detection_config = parse_ros_config(
        os.path.join(
            get_package_share_directory("speech"), "config", "doorbell_detection.yaml"
        ),
        [ModuleNames.HRI.value],
    )["doorbell_detection"]["ros__parameters"]

    env_type = os.environ.get("ENV_TYPE", "cpu")

    nodes = [
        Node(
            package="speech",
            executable="audio_capturer.py",
            ros_arguments=["--log-level", f"audio_capturer:={LOG_LEVEL}"],
            name="audio_capturer",
            output="screen",
            emulate_tty=True,
            parameters=[mic_config],
        ),
        Node(
            package="speech",
            executable="noise_cancellation.py",
            ros_arguments=["--log-level", f"noise_cancellation:={LOG_LEVEL}"],
            name="noise_cancellation",
            output="screen",
            emulate_tty=True,
            parameters=[noise_cancellation_config],
        ),
        Node(
            package="speech",
            executable="voice_detection.py",
            ros_arguments=["--log-level", f"voice_detection:={LOG_LEVEL}"],
            name="voice_detection",
            output="screen",
            emulate_tty=True,
            parameters=[voice_detection_config],
        ),
        # Door-event detection: only the DSP doorbell node for now (knock and the
        # Edge Impulse door model are intentionally not launched).
        Node(
            package="speech",
            executable="doorbell_detection.py",
            ros_arguments=["--log-level", f"doorbell_detection:={LOG_LEVEL}"],
            name="doorbell_detection",
            output="screen",
            emulate_tty=True,
            parameters=[doorbell_detection_config],
        ),
        Node(
            package="speech",
            executable="hear_streaming.py",
            ros_arguments=["--log-level", f"hear:={LOG_LEVEL}"],
            name="hear",
            output="screen",
            emulate_tty=True,
            parameters=[hear_streaming_config],
        ),
        Node(
            package="speech",
            executable="say.py",
            ros_arguments=["--log-level", f"say:={LOG_LEVEL}"],
            name="say",
            output="screen",
            emulate_tty=True,
            parameters=[speaker_config],
        ),
        Node(
            package="speech",
            executable="audio_feedback.py",
            ros_arguments=["--log-level", f"audio_feedback:={LOG_LEVEL}"],
            name="audio_feedback",
        ),
    ]

    if env_type == "l4t":
        eim_config = parse_ros_config(
            os.path.join(
                get_package_share_directory("speech"), "config", "kws_eim.yaml"
            ),
            [ModuleNames.HRI.value],
        )["kws_eim"]["ros__parameters"]

        # Doorbell detection is handled by the gated DSP node (doorbell_detection);
        # the Edge Impulse door model (door_eim) is intentionally not launched — it
        # ran ungated and generalised poorly to unseen, per-round doorbell sounds.
        nodes.extend(
            [
                Node(
                    package="speech",
                    executable="ei_audio_node.py",
                    ros_arguments=["--log-level", f"kws_eim:={LOG_LEVEL}"],
                    name="kws_eim",
                    output="screen",
                    emulate_tty=True,
                    parameters=[eim_config],
                ),
            ]
        )
    else:
        oww_config_path = os.path.join(
            get_package_share_directory("speech"), "config", "kws_oww.yaml"
        )
        oww_config = parse_ros_config(oww_config_path, [ModuleNames.HRI.value])[
            "kws_oww"
        ]["ros__parameters"]

        nodes.append(
            Node(
                package="speech",
                executable="kws_oww.py",
                ros_arguments=["--log-level", f"kws_oww:={LOG_LEVEL}"],
                name="kws_oww",
                output="screen",
                emulate_tty=True,
                parameters=[oww_config],
            )
        )

    if SpeechApiUtils.respeaker_available():
        nodes.append(
            Node(
                package="speech",
                executable="respeaker.py",
                ros_arguments=["--log-level", f"respeaker:={LOG_LEVEL}"],
                name="respeaker",
                output="screen",
                emulate_tty=True,
                parameters=[respeaker_config],
            )
        )

    return LaunchDescription(nodes)
