#!/usr/bin/env python3

import os
import subprocess

import rclpy
from gtts import gTTS
from pygame import mixer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils
from speech.wav_utils import WavUtils
from std_msgs.msg import Bool

from frida_constants.hri_constants import SPEAK_SERVICE
from frida_interfaces.srv import Speak

CURRENT_FILE_PATH = os.path.abspath(__file__)

FILE_DIR = CURRENT_FILE_PATH[: CURRENT_FILE_PATH.index("install")]
ASSETS_DIR = os.path.join(
    FILE_DIR, "src", "hri", "packages", "speech", "assets", "downloads"
)

VOICE_DIRECTORY = os.path.join(ASSETS_DIR, "offline_voice")

os.makedirs(VOICE_DIRECTORY, exist_ok=True)


class Say(Node):
    def __init__(self):
        super().__init__("say")
        self.get_logger().info("Initializing Say node.")

        self.connected = False
        self.declare_parameter("speaking_topic", "/saying")

        self.declare_parameter("SPEAK_SERVICE", SPEAK_SERVICE)
        self.declare_parameter("model", "en_US-amy-medium")
        self.declare_parameter("offline", True)

        self.declare_parameter("SPEAKER_DEVICE_NAME", "default")
        self.declare_parameter("SPEAKER_INPUT_CHANNELS", 32)
        self.declare_parameter("SPEAKER_OUT_CHANNELS", 32)

        speaker_device_name = (
            self.get_parameter("SPEAKER_DEVICE_NAME").get_parameter_value().string_value
        )
        speaker_input_channels = (
            self.get_parameter("SPEAKER_INPUT_CHANNELS")
            .get_parameter_value()
            .integer_value
        )
        speaker_out_channels = (
            self.get_parameter("SPEAKER_OUT_CHANNELS")
            .get_parameter_value()
            .integer_value
        )

        self.output_device_index = SpeechApiUtils.getIndexByNameAndChannels(
            speaker_device_name, speaker_input_channels, speaker_out_channels
        )

        speak_service = (
            self.get_parameter("SPEAK_SERVICE").get_parameter_value().string_value
        )

        speaking_topic = (
            self.get_parameter("speaking_topic").get_parameter_value().string_value
        )

        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.offline = self.get_parameter("offline").get_parameter_value().bool_value

        if not self.offline:
            self.connected = SpeechApiUtils.is_connected()

        self.create_service(Speak, speak_service, self.speak_service)
        self.publisher_ = self.create_publisher(Bool, speaking_topic, 10)

        self.get_logger().info("Say node initialized.")

    def speak_service(self, req, res):
        self.get_logger().info("[Service] I will say: " + req.text)
        if req.text:
            self.say(req.text)
            res.success = True
        else:
            res.success = False
            self.get_logger().info("[Service] Nothing to say.")

        return res

    def say(self, text):
        self.publisher_.publish(Bool(data=True))
        success = False
        try:
            if self.offline or not self.connected:
                self.offline_voice(text)
            else:
                self.connectedVoice(text)
            success = True
        except Exception as e:
            self.get_logger().error(e)
            if not self.offline:
                self.get_logger().warn("Retrying with offline mode")
                self.offline_voice(text)

        self.publisher_.publish(Bool(data=False))
        return success

    def offline_voice(self, text):
        text_chunks = self.split_text(text, 4000)
        counter = 0

        # Generate all wav files for each chunk
        for chunk in text_chunks:
            counter += 1
            output_path = os.path.join(VOICE_DIRECTORY, f"{counter}.wav")
            self.synthesize_voice_offline(output_path, chunk)

        self.get_logger().debug(f"Generated {counter} wav files.")

        # Play and remove all mp3 files
        for i in range(1, counter + 1):
            save_path = os.path.join(VOICE_DIRECTORY, f"{i}.wav")
            # WavUtils.play_wav(save_path, device_index=self.output_device_index)
            self.play_audio(save_path)
            WavUtils.discard_wav(save_path)

    def connectedVoice(self, text):
        tts = gTTS(text=text, lang="en")
        save_path = "play.mp3"
        tts.save(save_path)
        self.get_logger().info("Saying...")
        # WavUtils.play_mp3(save_path, device_index=self.output_device_index)
        # .play_mp3(save_path, device_index=self.output_device_index)
        self.play_audio(save_path)
        self.get_logger().info("Finished speaking.")

    def play_audio(self, file_path):
        mixer.pre_init(frequency=48000, buffer=2048)
        mixer.init()
        mixer.music.load(file_path)
        mixer.music.play()
        while mixer.music.get_busy():
            pass

    @staticmethod
    def split_text(text: str, max_len, split_sentences=False):
        """Split text into chunks of max_len characters. The model may not be able to synthesize long texts."""
        text_chunks = text.split(".") if split_sentences else [text]
        limited_chunks = []
        for chunk in text_chunks:
            while len(chunk) > 0:
                limited_chunks.append(chunk[:max_len])
                chunk = chunk[max_len:]

        return limited_chunks

    def synthesize_voice_offline(self, output_path, text):
        """Synthesize text using the offline voice model."""

        executable = (
            "/workspace/piper/install/piper"
            if os.path.exists("/workspace/piper/install/piper")
            else "python3.10 -m piper"
        )

        model_path = os.path.join(VOICE_DIRECTORY, self.model + ".onnx")

        download_model = not os.path.exists(model_path)

        if download_model:
            self.get_logger().info("Downloading voice model...")

        command = [
            "echo",
            f'"{text}"',
            "|",
            executable,
            "--model",
            self.model if download_model else model_path,
            "--data-dir",
            VOICE_DIRECTORY,
            "--download-dir",
            VOICE_DIRECTORY,
            "--output_file",
            output_path,
        ]

        subprocess.run(" ".join(command), shell=True)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(Say())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
