#!/usr/bin/env python3

import json
import os
import sys
import subprocess
from collections import OrderedDict

import rclpy
from gtts import gTTS
from pygame import mixer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils
from std_msgs.msg import Bool, String

from frida_constants.hri_constants import SPEAK_SERVICE
from frida_interfaces.srv import Speak

import grpc

# Add the directory containing the protos to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), "tts"))

import tts_pb2
import tts_pb2_grpc

CURRENT_FILE_PATH = os.path.abspath(__file__)

FILE_DIR = CURRENT_FILE_PATH[: CURRENT_FILE_PATH.index("install")]
ASSETS_DIR = os.path.join(
    FILE_DIR, "src", "hri", "packages", "speech", "assets", "downloads"
)

VOICE_DIRECTORY = os.path.join(ASSETS_DIR, "offline_voice")
CACHE_FILE = os.path.join(VOICE_DIRECTORY, "audio_cache.json")

os.makedirs(VOICE_DIRECTORY, exist_ok=True)


class Say(Node):
    def __init__(self):
        super().__init__("say")
        self.get_logger().info("Initializing Say node.")

        # Initialize LRU cache for audio files
        self.declare_parameter("cache_size", 100)
        self._cache_size = (
            self.get_parameter("cache_size").get_parameter_value().integer_value
        )
        self._audio_cache = OrderedDict()  # text -> filepath mapping

        # Load cached mappings from disk if they exist
        self._load_cache()

        self.connected = False
        self.declare_parameter("speaking_topic", "/saying")
        self.declare_parameter("text_spoken", "/speech/text_spoken")

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
        text_spoken = (
            self.get_parameter("text_spoken").get_parameter_value().string_value
        )

        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.offline = self.get_parameter("offline").get_parameter_value().bool_value

        if not self.offline:
            self.connected = SpeechApiUtils.is_connected()

        self.create_service(Speak, speak_service, self.speak_service)
        self.publisher_ = self.create_publisher(Bool, speaking_topic, 10)
        self.text_publisher_ = self.create_publisher(String, text_spoken, 10)

        self.get_logger().info("Say node initialized.")

    def _load_cache(self):
        """Load the cache from disk if it exists."""
        try:
            if os.path.exists(CACHE_FILE):
                with open(CACHE_FILE, "r") as f:
                    cache_data = json.load(f)
                    for text, filepath in cache_data.items():
                        if os.path.exists(filepath):
                            self._audio_cache[text] = filepath
                        else:
                            self.get_logger().warn(f"Cached file not found: {filepath}")
                self.get_logger().info(
                    f"Loaded {len(self._audio_cache)} entries from cache"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to load cache: {e}")
            self._audio_cache.clear()

    def _save_cache(self):
        """Save the current cache to disk."""
        try:
            with open(CACHE_FILE, "w") as f:
                json.dump(dict(self._audio_cache), f)
            self.get_logger().info(f"Saved {len(self._audio_cache)} entries to cache")
        except Exception as e:
            self.get_logger().error(f"Failed to save cache: {e}")

    def _get_cached_audio(self, text):
        """Check if text exists in cache and return its filepath."""
        if text in self._audio_cache:
            # Move to end to mark as recently used
            filepath = self._audio_cache.pop(text)
            self._audio_cache[text] = filepath
            return filepath
        return None

    def _add_to_cache(self, text, filepath):
        """Add a new text-filepath pair to the cache."""
        if len(self._audio_cache) >= self._cache_size:
            # Remove oldest entry (first item in OrderedDict)
            _, old_filepath = self._audio_cache.popitem(last=False)
            if os.path.exists(old_filepath):
                os.remove(old_filepath)

        self._audio_cache[text] = filepath
        self._save_cache()

    def speak_service(self, req, res):
        self.get_logger().info("[Service] I will say: " + req.text)
        if req.text:
            msg = String()
            msg.data = req.text
            self.text_publisher_.publish(msg)
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

        # Wait for audio to finish playing before returning
        try:
            while mixer.music.get_busy():
                pass
        except Exception as e:
            self.get_logger().error(f"Error while waiting for audio: {e}")
        self.publisher_.publish(Bool(data=False))
        return success

    def offline_voice(self, text):
        text_chunks = self.split_text(text, 4000)
        audio_files = []

        # Check cache or generate new files for each chunk
        for i, chunk in enumerate(text_chunks, 1):
            cached_path = self._get_cached_audio(chunk)
            if cached_path and os.path.exists(cached_path):
                self.get_logger().debug(f"Using cached audio for chunk {i}")
                audio_files.append(cached_path)
            else:
                output_path = os.path.join(VOICE_DIRECTORY, f"{hash(chunk)}.wav")
                self.synthesize_voice_offline(f"{hash(chunk)}.wav", chunk)
                self._add_to_cache(chunk, output_path)
                audio_files.append(output_path)

        self.get_logger().debug(f"Playing {len(audio_files)} audio files")

        # Play all audio files
        for filepath in audio_files:
            self.play_audio(filepath)

    def connectedVoice(self, text):
        cached_path = self._get_cached_audio(text)
        if cached_path and os.path.exists(cached_path):
            self.get_logger().info("Using cached audio")
            self.play_audio(cached_path)
            return

        save_path = os.path.join(VOICE_DIRECTORY, f"{hash(text)}.mp3")
        tts = gTTS(text=text, lang="es")
        tts.save(save_path)
        self._add_to_cache(text, save_path)
        self.get_logger().info("Saying...")
        self.play_audio(save_path)

    def play_audio(self, file_path):
        mixer.pre_init(frequency=48000, buffer=2048)
        mixer.init()
        while mixer.music.get_busy():
            pass
        mixer.music.load(file_path)
        mixer.music.play()

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
        try:
            with grpc.insecure_channel('127.0.0.1:50050') as channel:
                stub = tts_pb2_grpc.TTSServiceStub(channel)
                response = stub.Synthesize(tts_pb2.SynthesizeRequest(
                    text=text,
                    model=self.model,
                    output_path=output_path
                ))

            if not response.success:
                raise RuntimeError(f"TTS failed: {response.error_message}")
        except Exception as e:
            self.get_logger().error(f"[gRPC] Synthesis failed: {e}")
            raise



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
