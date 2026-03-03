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
        self.declare_parameter("CHUNK_SIZE", 1024)

        self.chunk_size = self.get_parameter("CHUNK_SIZE").value
        self.use_respeaker = SpeechApiUtils.respeaker_available()
        self.RATE = 16000

        self.publisher_ = self.create_publisher(
            AudioData, self.get_parameter("publish_topic").value, 20
        )

        mic_name = self.get_parameter("MIC_DEVICE_NAME").value
        in_ch = self.get_parameter("MIC_INPUT_CHANNELS").value
        out_ch = self.get_parameter("MIC_OUT_CHANNELS").value
        self.input_device_index = SpeechApiUtils.getIndexByNameAndChannels(
            mic_name, in_ch, out_ch
        )

    def record(self):
        p = pyaudio.PyAudio()
        channels = 6 if self.use_respeaker else 1

        try:
            stream = p.open(
                input_device_index=self.input_device_index,
                format=pyaudio.paInt16,
                channels=channels,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.chunk_size,
            )

            self.get_logger().info(
                f"--- Listening on device index {self.input_device_index} ---"
            )

            while rclpy.ok():
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                if not data:
                    continue

                audio_arr = np.frombuffer(data, dtype=np.int16)

                # If ReSpeaker, extract channel 0 (main microphone)
                if self.use_respeaker:
                    audio_arr = audio_arr[0::6]

                self.publisher_.publish(AudioData(data=audio_arr.tobytes()))

        except Exception as e:
            self.get_logger().error(f"Critical error in PyAudio: {e}")
        finally:
            self.get_logger().info("Closing AudioCapturer...")
            try:
                stream.stop_stream()
                stream.close()
            except Exception:
                pass
            p.terminate()


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
