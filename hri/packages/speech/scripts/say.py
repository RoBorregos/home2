#!/usr/bin/env python3

import json
import os
import sys
import time
from collections import OrderedDict

import numpy as np
import grpc
import rclpy
import soundfile as sf
from gtts import gTTS
from pygame import mixer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils
from std_msgs.msg import Bool, String
import wave

from frida_constants.hri_constants import SPEAK_SERVICE
from frida_interfaces.msg import AudioData
from frida_interfaces.srv import Speak

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
        self._audio_cache = OrderedDict()  # (text, speed) -> filepath mapping

        # Load cached mappings from disk if they exist
        self._load_cache()

        self.connected = False
        self.declare_parameter("speaking_topic", "/saying")
        self.declare_parameter("text_spoken", "/speech/text_spoken")
        self.declare_parameter("robot_audio_topic", "/robot_audio_output")

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
        robot_audio_topic = (
            self.get_parameter("robot_audio_topic").get_parameter_value().string_value
        )

        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.offline = self.get_parameter("offline").get_parameter_value().bool_value
        self.server_ip = (
            self.get_parameter("TTS_SERVER_IP").get_parameter_value().string_value
        )

        if not self.offline:
            self.connected = SpeechApiUtils.is_connected()

        self.create_service(Speak, speak_service, self.speak_service)
        self.publisher_ = self.create_publisher(Bool, speaking_topic, 10)
        self.text_publisher_ = self.create_publisher(String, text_spoken, 10)
        self.robot_audio_publisher_ = self.create_publisher(
            AudioData, robot_audio_topic, 10
        )

        self.get_logger().info("Say node initialized.")

    def interrupt_callback(self, msg):
        """Handle speech interrupt signal."""
        if msg.data == "interrupt":
            self.get_logger().info("Speech interrupted by user!")
            self.speech_interrupted = True
            # Stop current audio playback
            if mixer.get_init():
                mixer.music.stop()

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
                self.say(req.text, req.speed)
                res.success = True
            else:
                res.success = False
                self.get_logger().info("[Service] Nothing to say.")
        except Exception as e:
            self.get_logger().error(f"[Service] Error in speak_service: {e}")
        finally:
            return res

    def say(self, text, speed):
        self.publisher_.publish(Bool(data=True))
        success = False

        try:
            if self.offline or not self.connected:
                self.offline_voice(text, speed)
            else:
                self.connectedVoice(text)
            success = True
        except Exception as e:
            self.get_logger().error(e)
            if not self.offline:
                self.get_logger().warn("Retrying with offline mode")
                self.offline_voice(text, speed)

        self.publisher_.publish(Bool(data=False))
        return success

    def offline_voice(self, text, speed=1):
        text_chunks = self.split_text(text, 4000)
        audio_files = []

        # Check cache or generate new files for each chunk
        for i, chunk in enumerate(text_chunks, 1):
            cached_path = self._get_cached_audio(chunk, speed)
            if cached_path and os.path.exists(cached_path):
                self.get_logger().debug(f"Using cached audio for chunk {i}")
                audio_files.append(cached_path)

                self.get_logger().debug(f"Playing {len(audio_files)} audio files")

                # Play all audio files
                for filepath in audio_files:
                    self.play_audio(filepath)
            else:
                output_path = os.path.join(
                    VOICE_DIRECTORY, f"{hash((chunk, speed))}.wav"
                )
                self.synthesize_voice_offline(
                    f"{hash((chunk, speed))}.wav", chunk, speed
                )
                self._add_to_cache(chunk, output_path, speed)
                mixer.pre_init(frequency=48000, buffer=2048)
                mixer.init()
                while mixer.music.get_busy():
                    pass

    def connectedVoice(self, text):
        # For connected voice, speed is not applicable, so we use default speed=1.0
        cached_path = self._get_cached_audio(text, 1.0)
        if cached_path and os.path.exists(cached_path):
            self.get_logger().info("Using cached audio")
            self.play_audio(cached_path)
            return

        save_path = os.path.join(VOICE_DIRECTORY, f"{hash((text, 1.0))}.mp3")
        tts = gTTS(text=text, lang="es")
        tts.save(save_path)
        self._add_to_cache(text, save_path, 1.0)
        self.get_logger().info("Saying...")
        self.play_audio(save_path)

    def play_audio(self, file_path):
        try:
            audio_data, sample_rate = sf.read(file_path, dtype="int16")

            # If stereo, convert to mono
            if len(audio_data.shape) > 1:
                audio_data = np.mean(audio_data, axis=1).astype(np.int16)

            # Resample to 16kHz if needed (for AEC compatibility)
            if sample_rate != 16000:
                from scipy.signal import resample_poly
                from math import gcd

                g = gcd(sample_rate, 16000)
                up = 16000 // g
                down = sample_rate // g
                audio_data = resample_poly(audio_data, up, down).astype(np.int16)

            # Publish robot audio in chunks for real-time AEC
            chunk_size = 512  # Match typical audio chunk size
            for i in range(0, len(audio_data), chunk_size):
                chunk = audio_data[i : i + chunk_size]
                self.robot_audio_publisher_.publish(AudioData(data=chunk.tobytes()))

        except Exception as e:
            self.get_logger().warn(f"Could not read audio file for AEC: {e}")

        mixer.pre_init(frequency=48000, buffer=2048)
        mixer.init()
        while mixer.music.get_busy():
            pass
        mixer.music.load(file_path)
        mixer.music.play()

        # Wait for audio to finish playing or be interrupted
        try:
            while mixer.music.get_busy() and not self.speech_interrupted:
                time.sleep(0.01)  # Small delay to allow interrupt detection
        except Exception as e:
            self.get_logger().error(f"Error while waiting for audio: {e}")

    def publish_audio_reference(self, file_path):
        """Publish audio data as reference for AEC."""
        try:
            # Read audio file
            with wave.open(file_path, "rb") as wav_file:
                sample_rate = wav_file.getframerate()
                channels = wav_file.getnchannels()
                sample_width = wav_file.getsampwidth()

                # Read audio data
                audio_data = wav_file.readframes(wav_file.getnframes())

                # Convert to numpy array
                if sample_width == 2:
                    audio_array = np.frombuffer(audio_data, dtype=np.int16)
                else:
                    self.get_logger().warn(f"Unsupported sample width: {sample_width}")
                    return

                # Convert to mono if stereo
                if channels == 2:
                    audio_array = audio_array[::2]  # Take every other sample

                # Resample to 16kHz if needed (simple decimation for demo)
                if sample_rate != 16000:
                    downsample_factor = sample_rate // 16000
                    if downsample_factor > 1:
                        audio_array = audio_array[::downsample_factor]

                # Publish in chunks
                chunk_size = 512
                for i in range(0, len(audio_array), chunk_size):
                    if self.speech_interrupted:
                        break

                    chunk = audio_array[i : i + chunk_size]
                    if len(chunk) < chunk_size:
                        # Pad with zeros if needed
                        chunk = np.pad(chunk, (0, chunk_size - len(chunk)))

                    # Publish chunk
                    audio_msg = AudioData(data=chunk.tobytes())
                    self.far_audio_publisher.publish(audio_msg)

                    # Small delay to match real-time playback
                    time.sleep(chunk_size / 16000.0)

        except Exception as e:
            self.get_logger().error(f"Error publishing audio reference: {e}")

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
