#!/usr/bin/env python3

import os
import sys
import wave
import grpc

import numpy as np
import pyaudio
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils

from frida_interfaces.msg import AudioData
sys.path.append(os.path.join(os.path.dirname(__file__), "stt"))
import threading

import speech_pb2
import speech_pb2_grpc

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
        
        self.input_device_index = None

        self.get_logger().info("Input device index: " + str(self.input_device_index))

        if self.input_device_index is None:
            self.get_logger().warn(
                "Input device index not found, using system default."
            )

        self.get_logger().info("AudioCapturer node initialized.")

    def record(self):
        self.get_logger().info("AudioCapturer node recording.")
        iteration_step = 0
        CHUNK_SIZE = 512
        self.FORMAT = pyaudio.paInt16
        self.debug = True
        CHANNELS = 6 if self.use_respeaker else 1
        self.RATE = 16000
        EXTRACT_CHANNEL = 0

        self.p = pyaudio.PyAudio()
        stream = self.p.open(
            input_device_index=self.input_device_index,
            format=self.FORMAT,
            channels=CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=CHUNK_SIZE,
        )

        # GRPC client setup
        grpc_channel = grpc.insecure_channel('localhost:50051')
        stub = speech_pb2_grpc.SpeechStreamStub(grpc_channel)

        stop_flag = threading.Event()

        def request_generator():
            while not stop_flag.is_set() and stream.is_active() and rclpy.ok():
                try:
                    in_data = stream.read(CHUNK_SIZE, exception_on_overflow=False)

                    if self.use_respeaker:
                        in_data = np.frombuffer(in_data, dtype=np.int16)[EXTRACT_CHANNEL::6]
                        in_data = in_data.tobytes()

                    local_audio = bytes(in_data)
                    run_frames.append(local_audio)
                    ros_audio = bytes(local_audio)
                    self.publisher_.publish(AudioData(data=ros_audio))
                    grpc_audio = bytes(local_audio)
                    yield speech_pb2.AudioRequest(audio_data=grpc_audio)

                    nonlocal iteration_step
                    iteration_step += 1
                    if iteration_step % SAVE_IT == 0:
                        iteration_step = 0
                        self.save_audio()

                except IOError as e:
                    self.get_logger().error(f"I/O error({e.errno}): {e.strerror}")
                    break

        def handle_transcripts(responses):
            try:
                for response in responses:
                    self.get_logger().info(f"Transcript: {response.text}")
                    # Optionally publish to ROS topic here if needed
            except grpc.RpcError as e:
                self.get_logger().error(f"gRPC stream error: {e}")

        # Start receiving responses in a background thread
        
        responses = stub.Transcribe(request_generator())
        response_thread = threading.Thread(target=handle_transcripts, args=(responses,))
        response_thread.start()

        try:
            while stream.is_active() and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().info("Stopping on user interrupt.")
        finally:
            stop_flag.set()
            stream.stop_stream()
            stream.close()
            self.p.terminate()
            response_thread.join()
            self.get_logger().info("Audio stream closed.")

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
