import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from speech.speech_api_utils import SpeechApiUtils

from frida_constants import ModuleNames, parse_ros_config


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
            name="audio_capturer",
            output="screen",
            emulate_tty=True,
            parameters=[mic_config],
        ),
        Node(
            package="speech",
            executable="noise_cancellation.py",
            name="noise_cancellation",
            output="screen",
            emulate_tty=True,
            parameters=[noise_cancellation_config],
        ),
        Node(
            package="speech",
            executable="voice_detection.py",
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
            name="doorbell_detection",
            output="screen",
            emulate_tty=True,
            parameters=[doorbell_detection_config],
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
            executable="audio_feedback.py",
            name="audio_feedback",
        ),
    ]

    actions = [LogInfo(msg=f"Environment type detected: {env_type}")]

    if env_type == "l4t":
        actions.append(
            LogInfo(msg="L4T environment detected - adding Edge Impulse nodes")
        )
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
                    name="kws_eim",
                    output="screen",
                    emulate_tty=True,
                    parameters=[eim_config],
                ),
            ]
        )
    else:
        actions.append(
            LogInfo(msg=f"{env_type} environment detected - adding OpenWakeWord node")
        )
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
                name="kws_oww",
                output="screen",
                emulate_tty=True,
                parameters=[oww_config],
            )
        )

    if SpeechApiUtils.respeaker_available():
        actions.append(
            LogInfo(msg="ReSpeaker detected - adding respeaker node to launch")
        )
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
        actions.append(LogInfo(msg="ReSpeaker not detected - skipping respeaker node"))

    return LaunchDescription(nodes + actions)
