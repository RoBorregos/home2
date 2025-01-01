#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, Bool
from frida_interfaces.srv import Speak

import os
from gtts import gTTS
from pygame import mixer
import subprocess

from speech.speech_api_utils import SpeechApiUtils
from speech.wav_utils import WavUtils


SPEAK_SERVICE_TOPIC = "/speech/speak"
SPEAK_NOW_TOPIC = "/speech/speak_now"

# Offline voice
MODEL = "en_US-amy-medium"
CURRENT_FILE_PATH = os.path.abspath(__file__)

CURRENT_DIRECTORY = os.path.dirname(CURRENT_FILE_PATH)
VOICE_DIRECTORY = os.path.join(CURRENT_DIRECTORY, "offline_voice")


# Get device index using environment variables
SPEAKER_DEVICE_NAME = os.getenv("SPEAKER_DEVICE_NAME", default=None)
SPEAKER_INPUT_CHANNELS = int(os.getenv("SPEAKER_INPUT_CHANNELS", default=2))
SPEAKER_OUT_CHANNELS = int(os.getenv("SPEAKER_OUT_CHANNELS", default=0))

OUTPUT_DEVICE_INDEX = SpeechApiUtils.getIndexByNameAndChannels(
    SPEAKER_DEVICE_NAME, SPEAKER_INPUT_CHANNELS, SPEAKER_OUT_CHANNELS
)

if OUTPUT_DEVICE_INDEX is None:
    print("Warning: output device index not found, using system default.")

DEBUG = True
OFFLINE = True


class Say(Node):

    def __init__(self):
        super().__init__('say')
        self.publisher_ = self.create_publisher(Bool, 'saying', 10)
        self.get_logger().info('Say node has started.')

        self.connected = False
        if not OFFLINE:
            self.connected = SpeechApiUtils.is_connected()

        self.create_service(Speak, SPEAK_SERVICE_TOPIC, self.speak_service)

        self.create_subscription(String,
                                 SPEAK_NOW_TOPIC, self.speak_topic, 10)

    def speak_service(self, req):
        """When say is called as a service. Caller awaits for the response."""
        self.get_logger().debug("[Service] I will say: " + req.text)
        return self.say(req.text)

    def speak_topic(self, msg):
        """When say is called as a topic. Caller doesn't wait for response."""
        self.get_logger().debug("[Topic] I will say: " + msg.data)
        self.say(msg.data)

    def say(self, text):
        self.publisher_.publish(Bool(data=True))
        success = False
        try:
            if OFFLINE or not self.connected:
                self.offline_voice(text)
            else:
                self.connectedVoice(text)
            success = True
        except Exception as e:
            print(e)
            if not OFFLINE:
                print("Retrying with offline mode")
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

        print(f"Generated {counter} wav files.")

        # Play and remove all mp3 files
        for i in range(1, counter + 1):
            save_path = os.path.join(VOICE_DIRECTORY, f"{i}.wav")
            # WavUtils.play_wav(save_path, device_index=OUTPUT_DEVICE_INDEX)
            self.play_audio(save_path)
            WavUtils.discard_wav(save_path)

    def connectedVoice(self, text):
        tts = gTTS(text=text, lang="en")
        save_path = "play.mp3"
        tts.save(save_path)
        self.get_logger().info("Saying...")
        # WavUtils.play_mp3(save_path, device_index=OUTPUT_DEVICE_INDEX)
        # .play_mp3(save_path, device_index=OUTPUT_DEVICE_INDEX)
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

        executable = "/workspace/piper/install/piper" if os.path.exists(
            "/workspace/piper/install/piper") else "python3.10 -m piper"

        model_path = os.path.join(VOICE_DIRECTORY, MODEL + ".onnx")

        download_model = not os.path.exists(model_path)

        if download_model:
            self.get_logger().info("Downloading voice model...")

        command = [
            "echo",
            f'"{text}"',
            "|",
            executable,
            "--model",
            MODEL if download_model else model_path,
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


if __name__ == '__main__':
    main()
