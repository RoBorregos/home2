#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from frida_interfaces.msg import AudioData
from std_msgs.msg import Bool, String
from frida_interfaces.srv import STT, AudioText

from ament_index_python.packages import get_package_share_directory

import webrtcvad
import pyaudio
import collections

import numpy as np
import os
import onnxruntime


# Constants
FORMAT = pyaudio.paInt16
CHANNELS = 1
CHUNK_SIZE = 512
RATE = 16000
CHUNK_DURATION = CHUNK_SIZE / RATE
TIME_FOR_CHANGE = 0.25
COUNT_FOR_CHANGE = TIME_FOR_CHANGE / CHUNK_DURATION
MIN_AUDIO_LENGTH = 0.50
MIN_CHUNKS_AUDIO_LENGTH = MIN_AUDIO_LENGTH / CHUNK_DURATION

PADDING_DURATION = 0.50
NUM_PADDING_CHUNKS = int(PADDING_DURATION / CHUNK_DURATION)

current_dir = os.path.dirname(os.path.abspath(__file__))


class UsefulAudio(Node):
    def __init__(self):
        super().__init__("useful_audio")

        self.declare_parameter("debug", False)
        self.declare_parameter("USE_SILERO_VAD", True)
        self.declare_parameter("threshold", 0.1)
        self.declare_parameter("DISABLE_KWS", False)
        self.declare_parameter("MAX_AUDIO_DURATION", 10)
        self.declare_parameter("STT_SERVICE", False)

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
        self.stt_service = (
            self.get_parameter("STT_SERVICE").get_parameter_value().bool_value
        )

        self.triggered = False
        self.chunk_count = 0
        self.voiced_frames = []
        self.ring_buffer = collections.deque(maxlen=NUM_PADDING_CHUNKS)

        self.audio_collection = []
        self.chunk_collection = None

        self.timer = None
        self.is_saying = False
        self.audio_state = "None"
        self.service_active = False

        self.publisher = self.create_publisher(AudioData, "UsefulAudio", 20)
        self.audio_state_publisher = self.create_publisher(String, "AudioState", 10)
        self.lamp_publisher = self.create_publisher(String, "colorInstruction", 10)
        self.respeaker_light_publisher = self.create_publisher(
            String, "/ReSpeaker/light", 10
        )

        if self.stt_service:
            self.whisper_client = self.create_client(
                AudioText, "/speech/service/raw_command"
            )
            self.create_service(STT, "/speech/STT", self.stt_handler)

        self.create_subscription(
            AudioData, "rawAudioChunk", self.callback_raw_audio, 10
        )
        self.create_subscription(Bool, "saying", self.callback_saying, 10)
        self.create_subscription(Bool, "keyword_detected", self.callback_keyword, 10)

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
        self.triggered = True
        if not self.service_active:
            self.discard_audio()
            self.compute_audio_state()

    def compute_audio_state(self):
        new_state = (
            "saying" if self.is_saying else "listening" if self.triggered else "idle"
        )
        if self.audio_state != new_state:
            self.debug(f"Audio state changed from {self.audio_state} to {new_state}")
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

    def stt_handler(self, request, response):
        self.discard_audio()
        self.service_active = True
        self.triggered = True
        self.compute_audio_state()

        timeout = 15
        counter = 0
        while self.triggered and counter < timeout:
            self.get_clock().sleep_for(1)
            counter += 1

        raw_text = self.whisper_client.call(
            AudioData(data=self.voiced_frames)
        ).text_heard

        self.service_active = False
        self.discard_audio()
        return raw_text


def main(args=None):
    rclpy.init(args=args)
    useful_audio = UsefulAudio()
    try:
        rclpy.spin(useful_audio)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        useful_audio.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
