#!/usr/bin/env python3

import collections
import os

import numpy as np
import onnxruntime
import pyaudio
import rclpy
import webrtcvad
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from frida_interfaces.msg import AudioData

# Constants
FORMAT = pyaudio.paInt16  # Audio format (16-bit PCM)
CHANNELS = 1  # Number of audio channels (1 = mono)
CHUNK_SIZE = 512  # Number of audio frames per chunk
RATE = 16000  # Sampling rate in Hz (16 kHz)
CHUNK_DURATION = CHUNK_SIZE / RATE  # Duration of each chunk in seconds
TIME_FOR_CHANGE = 0.25  # Time in seconds for state change detection
COUNT_FOR_CHANGE = TIME_FOR_CHANGE / CHUNK_DURATION  # Number of chunks for state change
MIN_AUDIO_LENGTH = 0.50  # Minimum audio length in seconds
MIN_CHUNKS_AUDIO_LENGTH = (
    MIN_AUDIO_LENGTH / CHUNK_DURATION
)  # Minimum chunks for valid audio
PADDING_DURATION = 0.50  # Duration of padding audio in seconds
NUM_PADDING_CHUNKS = int(
    PADDING_DURATION / CHUNK_DURATION
)  # Number of chunks for padding
INT16_MAX_VALUE = 32768  # Maximum absolute value for int16 audio data
TRIGGER_THRESHOLD = 0.75  # Threshold for detecting speech or silence in ring buffer
NANOSECONDS_IN_SECOND = 1e9  # Conversion factor from nanoseconds to seconds

current_dir = os.path.dirname(os.path.abspath(__file__))


class UsefulAudio(Node):
    def __init__(self):
        super().__init__("useful_audio")

        self.declare_parameter("debug", False)
        self.declare_parameter("USE_SILERO_VAD", True)
        self.declare_parameter("threshold", 0.1)
        self.declare_parameter("DISABLE_KWS", False)
        self.declare_parameter("MAX_AUDIO_DURATION", 10)

        self.debug_mode = self.get_parameter("debug").get_parameter_value().bool_value
        self.use_silero_vad = (
            self.get_parameter("USE_SILERO_VAD").get_parameter_value().bool_value
        )
        self.threshold = (
            self.get_parameter("threshold").get_parameter_value().double_value
        )
        self.disable_kws = (
            self.get_parameter("DISABLE_KWS").get_parameter_value().bool_value
        )
        self.max_audio_duration = (
            self.get_parameter("MAX_AUDIO_DURATION").get_parameter_value().integer_value
        )

        self.triggered = False
        self.chunk_count = 0
        self.voiced_frames = None
        self.ring_buffer = collections.deque(maxlen=NUM_PADDING_CHUNKS)

        self.audio_collection = []
        self.chunk_collection = None

        self.timer = None
        self.is_saying = False
        self.audio_state = "idle"

        self.publisher = self.create_publisher(AudioData, "UsefulAudio", 20)
        self.audio_state_publisher = self.create_publisher(String, "AudioState", 10)
        self.lamp_publisher = self.create_publisher(String, "colorInstruction", 10)
        self.respeaker_light_publisher = self.create_publisher(
            String, "/ReSpeaker/light", 10
        )

        self.create_subscription(
            AudioData, "rawAudioChunk", self.callback_raw_audio, 10
        )
        self.create_subscription(Bool, "saying", self.callback_saying, 10)
        self.create_subscription(
            String, "/wakeword_detected", self.callback_keyword, 10
        )

        if not self.use_silero_vad:
            self.vad = webrtcvad.Vad()
            self.vad.set_mode(3)
        else:
            package_share_directory = get_package_share_directory("speech")
            model_path = os.path.join(
                package_share_directory, "assets", "silero_vad.onnx"
            )
            options = onnxruntime.SessionOptions()
            options.log_severity_level = 4

            self.inference_session = onnxruntime.InferenceSession(
                model_path, sess_options=options
            )
            self.sampling_rate = RATE
            self.h = np.zeros((2, 1, 64), dtype=np.float32)
            self.c = np.zeros((2, 1, 64), dtype=np.float32)

        self.log("UsefulAudio Initialized.")

    def log(self, message):
        self.get_logger().info(message)

    def debug(self, message):
        if self.debug_mode:
            self.log(message)

    def build_audio(self, data):
        if self.voiced_frames is None:
            self.voiced_frames = data
        else:
            self.voiced_frames += data
        self.chunk_count += 1

    def discard_audio(self):
        self.ring_buffer.clear()
        self.voiced_frames = None
        self.chunk_count = 0

    def publish_audio(self):
        if self.chunk_count > MIN_CHUNKS_AUDIO_LENGTH:
            self.publisher.publish(AudioData(data=self.voiced_frames))
        self.discard_audio()

    def int2float(self, sound):
        abs_max = np.abs(sound).max()
        sound = sound.astype("float32")
        if abs_max > 0:
            sound *= 1 / INT16_MAX_VALUE
        sound = sound.squeeze()
        return sound

    def is_speech_silero_vad(self, audio_data: np.ndarray) -> bool:
        input_data = {
            "input": audio_data.reshape(1, -1),
            "sr": np.array([self.sampling_rate], dtype=np.int64),
            "h": self.h,
            "c": self.c,
        }
        out, h, c = self.inference_session.run(None, input_data)
        self.h, self.c = h, c
        return out > self.threshold

    def vad_collector(self, chunk):
        is_speech = False

        if self.use_silero_vad:
            chunk_vad = np.frombuffer(chunk, dtype=np.int16)
            audio_float32 = self.int2float(chunk_vad)

            if self.chunk_collection is None:
                self.chunk_collection = chunk
            else:
                self.chunk_collection += chunk

            if len(self.audio_collection) < 3:
                self.audio_collection.append(audio_float32)
                return

            audio = np.concatenate(self.audio_collection)
            chunk = self.chunk_collection
            self.audio_collection = []
            self.chunk_collection = None
            is_speech = self.is_speech_silero_vad(audio)
        else:
            is_speech = self.vad.is_speech(chunk, RATE)

        if not self.triggered:
            self.ring_buffer.append((chunk, is_speech))
            num_voiced = len([f for f, speech in self.ring_buffer if speech])

            if num_voiced > TRIGGER_THRESHOLD * self.ring_buffer.maxlen:
                self.triggered = True
                self.compute_audio_state()
                self.debug("Moving to triggered state")
                for f, _ in self.ring_buffer:
                    self.build_audio(f)
                self.ring_buffer.clear()
        else:
            if self.timer is None:
                self.timer = self.get_clock().now()

            self.build_audio(chunk)
            self.ring_buffer.append((chunk, is_speech))
            num_unvoiced = len([f for f, speech in self.ring_buffer if not speech])

            current_time = self.get_clock().now()
            time_delta = (current_time - self.timer).nanoseconds / NANOSECONDS_IN_SECOND

            if (
                num_unvoiced > TRIGGER_THRESHOLD * self.ring_buffer.maxlen
                or time_delta > self.max_audio_duration
            ):
                self.triggered = False
                self.compute_audio_state()
                self.publish_audio()
                self.timer = None

    def callback_raw_audio(self, msg):
        if self.audio_state == "saying":
            self.discard_audio()
        elif self.audio_state == "listening" or self.disable_kws:
            self.vad_collector(msg.data)
        else:
            self.discard_audio()

    def callback_saying(self, msg):
        self.is_saying = msg.data
        self.compute_audio_state()

    def callback_keyword(self, msg):
        self.log("keyword callback activated.")
        self.triggered = True

        self.discard_audio()
        self.compute_audio_state()

        # if not self.service_active:
        #     self.discardAudio()
        #     self.computeAudioState()
        # if self.audio_state == "idle":
        #     self.triggered = True
        #     self.discard_audio()
        #     self.compute_audio_state()

    def compute_audio_state(self):
        new_state = (
            "saying" if self.is_saying else "listening" if self.triggered else "idle"
        )
        if self.audio_state != new_state:
            self.debug(f"Audio state changed from {self.audio_state} to {new_state}")
            self.timer = None
            self.audio_state = new_state
            self.audio_state_publisher.publish(String(data=self.audio_state))
            self.publish_lamp(new_state)
            self.publish_respeaker_light(new_state)

    def publish_lamp(self, state):
        color = (
            "GREEN" if state == "saying" else "BLUE" if state == "listening" else "RED"
        )
        self.lamp_publisher.publish(String(data=color))

    def publish_respeaker_light(self, state):
        light = (
            "speak" if state == "saying" else "think" if state == "listening" else "off"
        )
        self.respeaker_light_publisher.publish(String(data=light))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(UsefulAudio())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
