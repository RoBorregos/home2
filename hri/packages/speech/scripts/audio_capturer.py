#!/usr/bin/env python3

import os
import wave

import numpy as np
import pyaudio
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils

from frida_interfaces.msg import AudioData

SAVE_PATH = "/workspace/src/hri/packages/speech/debug/"
run_frames = []
SAVE_IT = 100


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

        self.get_logger().info("Input device index: " + str(self.input_device_index))

        if self.input_device_index is None:
            self.get_logger().warn(
                "Input device index not found, using system default."
            )

        self.get_logger().info("AudioCapturer node initialized.")

    def record(self):
        self.get_logger().info("AudioCapturer node recording.")
        iteration_step = 0
        # Format for the recorded audio, constants set from the Porcupine demo.py
        CHUNK_SIZE = 512
        self.FORMAT = pyaudio.paInt16  # Signed 2 bytes.
        self.debug = False
        CHANNELS = 6 if self.use_respeaker else 1
        self.RATE = 16000
        EXTRACT_CHANNEL = 0  # Use channel 0. Tested with TestMic.py. See channel meaning: https://wiki.seeedstudio.com/ReSpeaker-USB-Mic-Array/#update-firmware

        self.p = pyaudio.PyAudio()
        stream = self.p.open(
            input_device_index=self.input_device_index,  # See list_audio_devices() or set it to None for default
            format=self.FORMAT,
            channels=CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=CHUNK_SIZE,
        )

        while stream.is_active() and rclpy.ok():
            try:
                in_data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                iteration_step += 1

                if iteration_step % SAVE_IT == 0:
                    iteration_step = 0
                    self.save_audio()

                if self.use_respeaker:
                    in_data = np.frombuffer(in_data, dtype=np.int16)[EXTRACT_CHANNEL::6]
                    in_data = in_data.tobytes()
                msg = in_data
                run_frames.append(msg)
                self.publisher_.publish(AudioData(data=msg))
            except IOError as e:
                self.get_logger().error(
                    "I/O error({0}): {1}".format(e.errno, e.strerror)
                )

        if not stream.is_active():
            self.get_logger().error("Audio stream is not active.")

        stream.stop_stream()
        stream.close()
        self.p.terminate()

    def save_audio(self):
        if not self.debug:
            return

        self.get_logger().info("Saving audio stream.")
        output_file = os.path.join(SAVE_PATH, "last_run_audio.wav")

        with wave.open(output_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(self.p.get_sample_size(self.FORMAT))
            wf.setframerate(self.RATE)
            wf.writeframes(b"".join(run_frames))


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
