#!/usr/bin/env python3

import numpy as np
import pyaudio
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils

from frida_interfaces.msg import AudioData


class AudioCapturer(Node):
    def __init__(self):
        super().__init__("audio_capturer")

        self.declare_parameter("publish_topic", "/rawAudioChunk")
        self.declare_parameter("MIC_DEVICE_NAME", "default")
        self.declare_parameter("MIC_INPUT_CHANNELS", 32)
        self.declare_parameter("MIC_OUT_CHANNELS", 32)
        self.declare_parameter("USE_RESPEAKER", False)

        publish_topic = (
            self.get_parameter("publish_topic").get_parameter_value().string_value
        )

        self.use_respeaker = (
            self.get_parameter("USE_RESPEAKER").get_parameter_value().bool_value
        )

        mic_device_name = (
            self.get_parameter("MIC_DEVICE_NAME").get_parameter_value().string_value
        )
        mic_input_channels = (
            self.get_parameter("MIC_INPUT_CHANNELS").get_parameter_value().integer_value
        )
        mic_out_channels = (
            self.get_parameter("MIC_OUT_CHANNELS").get_parameter_value().integer_value
        )

        self.publisher_ = self.create_publisher(AudioData, publish_topic, 20)
        self.input_device_index = SpeechApiUtils.getIndexByNameAndChannels(
            mic_device_name, mic_input_channels, mic_out_channels
        )

        if self.input_device_index is None:
            self.get_logger().warn(
                "Input device index not found, using system default."
            )

        self.get_logger().info("AudioCapturer node initialized.")

    def record(self):
        self.get_logger().info("AudioCapturer node recording.")

        # Format for the recorded audio, constants set from the Porcupine demo.py
        CHUNK_SIZE = 512
        FORMAT = pyaudio.paInt16  # Signed 2 bytes.
        CHANNELS = 6 if self.use_respeaker else 1
        RATE = 16000 if self.use_respeaker else 41000
        EXTRACT_CHANNEL = 0  # Use channel 0. Tested with TestMic.py. See channel meaning: https://wiki.seeedstudio.com/ReSpeaker-USB-Mic-Array/#update-firmware

        p = pyaudio.PyAudio()
        stream = p.open(
            input_device_index=self.input_device_index,  # See list_audio_devices() or set it to None for default
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK_SIZE,
        )

        while stream.is_active() and rclpy.ok():
            try:
                in_data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                if self.use_respeaker:
                    in_data = np.frombuffer(in_data, dtype=np.int16)[EXTRACT_CHANNEL::6]
                    in_data = in_data.tobytes()
                msg = in_data
                self.publisher_.publish(AudioData(data=msg))
            except IOError as e:
                self.get_logger().error(
                    "I/O error({0}): {1}".format(e.errno, e.strerror)
                )

        if not stream.is_active():
            self.get_logger().error("Audio stream is not active.")

        stream.stop_stream()
        stream.close()
        p.terminate()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AudioCapturer()
        node.record()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
