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

SAVE_PATH = "/tmp"
SAVE_IT = 100
run_frames = []


class AudioCapturer(Node):
    def __init__(self):
        super().__init__("audio_capturer")

        self.declare_parameter("publish_topic", "/hri/rawAudioChunk")
        self.declare_parameter("MIC_DEVICE_NAME", "default")
        self.declare_parameter("MIC_INPUT_CHANNELS", 32)
        self.declare_parameter("MIC_OUT_CHANNELS", 32)
        self.declare_parameter("CHUNK_SIZE", 1024)
        self.declare_parameter("DEBUG", False)

        self.chunk_size = self.get_parameter("CHUNK_SIZE").value
        self.debug = self.get_parameter("DEBUG").value
        self.use_respeaker = SpeechApiUtils.respeaker_available()
        self.get_logger().info(f"ReSpeaker detected: {self.use_respeaker}")
        self.RATE = 16000

        self.publisher_ = self.create_publisher(
            AudioData, self.get_parameter("publish_topic").value, 20
        )

        mic_device_name = self.get_parameter("MIC_DEVICE_NAME").value
        mic_input_channels = self.get_parameter("MIC_INPUT_CHANNELS").value
        mic_out_channels = self.get_parameter("MIC_OUT_CHANNELS").value
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
        self.p = pyaudio.PyAudio()
        self.FORMAT = pyaudio.paInt16
        channels = 6 if self.use_respeaker else 1
        iteration_step = 0

        try:
            stream = self.p.open(
                input_device_index=self.input_device_index,
                format=self.FORMAT,
                channels=channels,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.chunk_size,
            )

            self.get_logger().info(
                f"--- Listening on device index {self.input_device_index} ---"
            )

            while rclpy.ok() and stream.is_active():
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                if not data:
                    continue

                # If ReSpeaker, extract channel 0 (main microphone), otherwise publish raw bytes
                if self.use_respeaker:
                    data = np.frombuffer(data, dtype=np.int16)[0::6].tobytes()

                self.publisher_.publish(AudioData(data=data))

                if self.debug:
                    run_frames.append(data)
                    iteration_step += 1
                    if iteration_step % SAVE_IT == 0:
                        iteration_step = 0
                        self.save_audio()

        except Exception as e:
            self.get_logger().error(f"Critical error in PyAudio: {e}")
        finally:
            self.get_logger().info("Closing AudioCapturer...")
            try:
                stream.stop_stream()
                stream.close()
            except Exception:
                pass
            self.p.terminate()

    def save_audio(self):
        self.get_logger().info("Saving audio stream.")
        output_file = os.path.join(SAVE_PATH, "last_run_audio.wav")

        with wave.open(output_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(self.p.get_sample_size(self.FORMAT))
            wf.setframerate(self.RATE)
            wf.writeframes(b"".join(run_frames))


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapturer()
    try:
        node.record()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
