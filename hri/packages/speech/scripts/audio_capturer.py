#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from frida_interfaces.msg import AudioData

import pyaudio
import os
import numpy as np

from speech.speech_api_utils import SpeechApiUtils


USE_RESPEAKER = False

# Get device index using environment variables
MIC_DEVICE_NAME = os.getenv("MIC_DEVICE_NAME", default=None)
MIC_INPUT_CHANNELS = int(os.getenv("MIC_INPUT_CHANNELS", default=2))
MIC_OUT_CHANNELS = int(os.getenv("MIC_OUT_CHANNELS", default=0))

INPUT_DEVICE_INDEX = SpeechApiUtils.getIndexByNameAndChannels(
    MIC_DEVICE_NAME, MIC_INPUT_CHANNELS, MIC_OUT_CHANNELS)

if INPUT_DEVICE_INDEX is None:
    print("Warning: input device index not found, using system default.")


class AudioCapturer(Node):
    def __init__(self):
        super().__init__('audio_capturer')
        self.publisher_ = self.create_publisher(AudioData, 'rawAudioChunk', 20)
        self.get_logger().info('AudioCapturer node initialized.')

    def record(self):
        self.get_logger().info('AudioCapturer node recording.')

        # Format for the recorded audio, constants set from the Porcupine demo.py
        CHUNK_SIZE = 512
        FORMAT = pyaudio.paInt16  # Signed 2 bytes.
        CHANNELS = 1 if not USE_RESPEAKER else 6
        RATE = 16000 if USE_RESPEAKER else 41000
        EXTRACT_CHANNEL = 0  # Use channel 0. Tested with TestMic.py. See channel meaning: https://wiki.seeedstudio.com/ReSpeaker-USB-Mic-Array/#update-firmware

        p = pyaudio.PyAudio()
        stream = p.open(input_device_index=INPUT_DEVICE_INDEX,  # See list_audio_devices() or set it to None for default
                        format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK_SIZE)

        while stream.is_active() and rclpy.ok():
            try:
                in_data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                if USE_RESPEAKER:
                    in_data = np.frombuffer(in_data, dtype=np.int16)[
                        EXTRACT_CHANNEL:: 6]
                    in_data = in_data.tobytes()
                msg = in_data
                self.publisher_.publish(AudioData(data=msg))
            except IOError as e:
                print("I/O error({0}): {1}".format(e.errno, e.strerror))

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


if __name__ == '__main__':
    main()
