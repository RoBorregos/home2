#!/usr/bin/env python3

import json
import os
from collections import OrderedDict

import grpc
import rclpy
from gtts import gTTS
from pygame import mixer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils
from std_msgs.msg import Bool, String

from frida_constants.hri_constants import SPEAK_SERVICE
from frida_interfaces.srv import Speak
from proto_interfaces import tts_pb2, tts_pb2_grpc

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
        self._audio_cache = OrderedDict()  # (text, speed) -> filepath mapping

        # Load cached mappings from disk if they exist
        self._load_cache()

        self.connected = False
        self.declare_parameter("speaking_topic", "/saying")
        self.declare_parameter("text_spoken", "/speech/text_spoken")

        self.declare_parameter("SPEAK_SERVICE", SPEAK_SERVICE)
        self.declare_parameter("model", "en_US-amy-medium")
        self.declare_parameter("offline", True)
        self.declare_parameter("TTS_SERVER_IP", "127.0.0.1:50050")

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
        self.server_ip = (
            self.get_parameter("TTS_SERVER_IP").get_parameter_value().string_value
        )

        if not self.offline:
            self.connected = SpeechApiUtils.is_connected()

        # Initialize pygame mixer
        mixer.pre_init(frequency=48000, buffer=2048)
        mixer.init()

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
                    for key, filepath in cache_data.items():
                        # Handle both old format (text only) and new format (text,speed)
                        if isinstance(key, str) and "," in key:
                            # New format: "text,speed"
                            text, speed_str = key.rsplit(",", 1)
                            try:
                                speed = float(speed_str)
                                cache_key = (text, speed)
                            except ValueError:
                                # Skip invalid entries
                                continue
                        else:
                            # Old format: text only, assume speed=1
                            cache_key = (key, 1.0)

                        if os.path.exists(filepath):
                            self._audio_cache[cache_key] = filepath
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
            # Convert cache keys (text, speed) to strings for JSON serialization
            serializable_cache = {}
            for (text, speed), filepath in self._audio_cache.items():
                key = f"{text},{speed}"
                serializable_cache[key] = filepath

            with open(CACHE_FILE, "w") as f:
                json.dump(serializable_cache, f)
            self.get_logger().info(f"Saved {len(self._audio_cache)} entries to cache")
        except Exception as e:
            self.get_logger().error(f"Failed to save cache: {e}")

    def _get_cached_audio(self, text, speed=1.0):
        """Check if text and speed combination exists in cache and return its filepath."""
        cache_key = (text, speed)
        if cache_key in self._audio_cache:
            # Move to end to mark as recently used
            filepath = self._audio_cache.pop(cache_key)
            self._audio_cache[cache_key] = filepath
            return filepath

        return None

    def _add_to_cache(self, text, filepath, speed=1.0):
        """Add a new text-filepath pair to the cache."""
        if len(self._audio_cache) >= self._cache_size:
            # Remove oldest entry (first item in OrderedDict)
            _, old_filepath = self._audio_cache.popitem(last=False)
            if os.path.exists(old_filepath):
                os.remove(old_filepath)

        cache_key = (text, speed)
        self._audio_cache[cache_key] = filepath
        self._save_cache()

    def speak_service(self, req, res):
        self.get_logger().info("[Service] text received: " + req.text)
        req.text = req.text.replace("*", "").replace("_", " ")

        try:
            self.get_logger().info("[Service] I will say: " + req.text)
            if req.text:
                msg = String()
                msg.data = req.text
                self.text_publisher_.publish(msg)
                success, duration = self.say(req.text, req.speed, req.wait)
                res.success = success
                res.duration = float(duration)
            else:
                res.success = False
                res.duration = 0.0
                self.get_logger().info("[Service] Nothing to say.")
        except Exception as e:
            self.get_logger().error(f"[Service] Error in speak_service: {e}")
            res.success = False
            res.duration = 0.0
        finally:
            return res

    def say(self, text, speed, wait=True):
        self.publisher_.publish(Bool(data=True))
        success = False
        duration = 0.0

        try:
            if self.offline or not self.connected:
                duration = self.offline_voice(text, speed, wait)
            else:
                duration = self.connectedVoice(text, wait)
            success = True
        except Exception as e:
            self.get_logger().error(e)
            if not self.offline:
                self.get_logger().warn("Retrying with offline mode")
                duration = self.offline_voice(text, speed, wait)
                success = True

        if wait:
            self.publisher_.publish(Bool(data=False))
        return success, duration

    def offline_voice(self, text, speed=1, wait=True):
        text_chunks = self.split_text(text, 4000)
        audio_files = []
        total_duration = 0.0

        # Check cache or generate new files for each chunk
        for i, chunk in enumerate(text_chunks, 1):
            cached_path = self._get_cached_audio(chunk, speed)
            if cached_path and os.path.exists(cached_path):
                self.get_logger().debug(f"Using cached audio for chunk {i}")
                audio_files.append(cached_path)
            else:
                output_path = os.path.join(
                    VOICE_DIRECTORY, f"audios/{hash((chunk, speed))}.wav"
                )
                self.synthesize_voice_offline(
                    f"{hash((chunk, speed))}.wav", chunk, speed
                )
                self._add_to_cache(chunk, output_path, speed)
                audio_files.append(output_path)

        self.get_logger().debug(f"Playing {len(audio_files)} audio files")

        # Play all audio files
        for i, filepath in enumerate(audio_files):
            # Only wait on the last file if wait=True, or wait on all if wait=True?
            # Actually, if we want to return the duration and start playing,
            # we should play all of them.
            # For simplicity, if wait is True, we block on each.
            # If wait is False, we might have an issue with multiple chunks.
            # But usually it's one chunk.
            is_last = i == len(audio_files) - 1
            chunk_duration = self.play_audio(filepath, wait=(wait and is_last))
            total_duration += chunk_duration

        return total_duration

    def connectedVoice(self, text, wait=True):
        # For connected voice, speed is not applicable, so we use default speed=1.0
        cached_path = self._get_cached_audio(text, 1.0)
        if not (cached_path and os.path.exists(cached_path)):
            cached_path = os.path.join(VOICE_DIRECTORY, f"{hash((text, 1.0))}.mp3")
            tts = gTTS(text=text, lang="es")
            tts.save(cached_path)
            self._add_to_cache(text, cached_path, 1.0)

        self.get_logger().info("Saying...")
        return self.play_audio(cached_path, wait=wait)

    def play_audio(self, file_path, wait=True):
        while mixer.music.get_busy():
            pass
        mixer.music.load(file_path)

        # Get duration
        try:
            sound = mixer.Sound(file_path)
            duration = sound.get_length()
        except Exception:
            # Fallback for MP3 or if Sound() fails
            duration = len(file_path) / 10.0  # Very rough fallback

        mixer.music.play()

        if wait:
            # Wait for audio to finish playing before returning
            try:
                while mixer.music.get_busy():
                    pass
            except Exception as e:
                self.get_logger().error(f"Error while waiting for audio: {e}")

        return duration

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

    def synthesize_voice_offline(self, output_path, text, speed):
        try:
            with grpc.insecure_channel(self.server_ip) as channel:
                stub = tts_pb2_grpc.TTSServiceStub(channel)
                response = stub.Synthesize(
                    tts_pb2.SynthesizeRequest(
                        text=text,
                        model=self.model,
                        output_path=output_path,
                        speed=speed,
                    )
                )

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
