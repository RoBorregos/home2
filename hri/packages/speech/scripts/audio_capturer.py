#!/usr/bin/env python3

import os
import numpy as np
import pyaudio
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils

from frida_interfaces.msg import AudioData
from speech.audio_processing import (
    reduce_noise,
    regulate_gain,
    compress_dynamic_range,
    adaptive_agc,
    dereverb_spectral,
)

SAVE_PATH = "/workspace/src/hri/packages/speech/debug/"
run_frames = []


class AudioCapturer(Node):
    def __init__(self):
        super().__init__("audio_capturer")

        self.declare_parameter("output_topic", "/processedAudioChunk")
        self.declare_parameter("MIC_DEVICE_NAME", "default")
        self.declare_parameter("MIC_INPUT_CHANNELS", 32)
        self.declare_parameter("MIC_OUT_CHANNELS", 32)
        self.declare_parameter("RESPEAKER_JOIN_METHOD", "avg")
        self.declare_parameter("PUBLISH_PER_MIC", False)
        self.declare_parameter("apply_processing", True)
        self.declare_parameter(
            "pipeline",
            ["reduce_noise", "regulate_gain", "compress", "agc", "dereverb"],
        )
        self.declare_parameter("debug_dir", "/tmp/speech_audio")

        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        self.use_respeaker = SpeechApiUtils.respeaker_available()
        self.get_logger().info(f"ReSpeaker detected: {self.use_respeaker}")

        mic_device_name = (
            self.get_parameter("MIC_DEVICE_NAME").get_parameter_value().string_value
        )
        mic_input_channels = (
            self.get_parameter("MIC_INPUT_CHANNELS").get_parameter_value().integer_value
        )
        mic_out_channels = (
            self.get_parameter("MIC_OUT_CHANNELS").get_parameter_value().integer_value
        )

        self.apply_processing = (
            self.get_parameter("apply_processing").get_parameter_value().bool_value
        )
        self.pipeline = (
            self.get_parameter("pipeline").get_parameter_value().string_array_value
        )
        self.debug_dir = (
            self.get_parameter("debug_dir").get_parameter_value().string_value
        )
        os.makedirs(self.debug_dir, exist_ok=True)

        self.publisher_ = self.create_publisher(AudioData, output_topic, 20)
        self.per_mic_publishers = []
        if self.get_parameter("PUBLISH_PER_MIC").get_parameter_value().bool_value:
            for i in range(mic_input_channels if mic_input_channels > 0 else 4):
                topic = output_topic + f"_mic{i}"
                self.per_mic_publishers.append(
                    self.create_publisher(AudioData, topic, 10)
                )

        self.input_device_index = SpeechApiUtils.getIndexByNameAndChannels(
            mic_device_name, mic_input_channels, mic_out_channels
        )

        self.get_logger().info("Input device index: " + str(self.input_device_index))

        if self.input_device_index is None:
            self.get_logger().warn(
                "Input device index not found, using system default."
            )

        self.mic_input_channels = mic_input_channels
        self.mic_out_channels = mic_out_channels
        self.join_method = (
            self.get_parameter("RESPEAKER_JOIN_METHOD")
            .get_parameter_value()
            .string_value
        )
        self.publish_per_mic = (
            self.get_parameter("PUBLISH_PER_MIC").get_parameter_value().bool_value
        )

        self.get_logger().info("AudioCapturer node initialized.")

    def record(self):
        self.get_logger().info("AudioCapturer node recording.")
        # iteration_step = 0
        CHUNK_SIZE = 512
        self.FORMAT = pyaudio.paInt16  # Signed 2 bytes.
        self.debug = False
        CHANNELS = 6 if self.use_respeaker else 1
        CHANNELS = self.mic_input_channels if self.mic_input_channels else CHANNELS
        self.RATE = 16000
        EXTRACT_CHANNEL = 0  # Use channel 0. Tested with microphone.py. See channel meaning: https://wiki.seeedstudio.com/ReSpeaker-USB-Mic-Array/#update-firmware

        self.p = pyaudio.PyAudio()
        stream = self.p.open(
            input_device_index=self.input_device_index,  # See list_audio_devices() or set it to None for default
            format=self.FORMAT,
            channels=CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=CHUNK_SIZE,
        )

        try:
            while stream.is_active() and rclpy.ok():
                in_data = stream.read(CHUNK_SIZE, exception_on_overflow=False)

                if self.use_respeaker:
                    arr = np.frombuffer(in_data, dtype=np.int16)
                    channels = (
                        self.mic_input_channels
                        if self.mic_input_channels and self.mic_input_channels > 0
                        else CHANNELS
                    )

                    if channels <= 1 or arr.size < channels:
                        mono = arr[EXTRACT_CHANNEL::6] if arr.size >= 6 else arr
                        local_audio = mono.tobytes()
                    else:
                        n_frames = arr.size // channels
                        usable = arr[: n_frames * channels]
                        frames = usable.reshape(n_frames, channels)

                        per_mics = [
                            frames[:, i].astype(np.int32)
                            for i in range(min(channels, frames.shape[1]))
                        ]

                        if self.publish_per_mic and len(self.per_mic_publishers) > 0:
                            for i, mic in enumerate(per_mics):
                                mic16 = mic.astype(np.int16).tobytes()
                                if i < len(self.per_mic_publishers):
                                    self.per_mic_publishers[i].publish(
                                        AudioData(data=mic16)
                                    )

                        if self.join_method == "sum":
                            mixed = np.sum(np.vstack(per_mics), axis=0)
                            mixed = np.clip(mixed, -32768, 32767).astype(np.int16)
                        elif self.join_method == "first":
                            mixed = per_mics[0].astype(np.int16)
                        else:
                            mixed = np.mean(np.vstack(per_mics), axis=0).astype(
                                np.int16
                            )

                        local_audio = mixed.tobytes()
                else:
                    local_audio = in_data

                ros_audio = bytes(local_audio)

                if self.apply_processing:
                    try:
                        arr16 = np.frombuffer(local_audio, dtype=np.int16)
                        y = arr16.astype(np.float32) / 32768.0

                        for step in self.pipeline:
                            try:
                                if step == "reduce_noise":
                                    y = reduce_noise(y, self.RATE)
                                elif step == "regulate_gain":
                                    y = regulate_gain(y, target_rms=0.05)
                                elif step == "compress":
                                    y = compress_dynamic_range(
                                        y, threshold_db=-20.0, ratio=2.0
                                    )
                                elif step == "agc":
                                    y, _ = adaptive_agc(y, target_rms=0.05)
                                elif step == "dereverb":
                                    y = dereverb_spectral(y, self.RATE)
                                else:
                                    pass
                            except Exception as e:
                                self.get_logger().warning(
                                    f"Processing step {step} failed: {e}"
                                )

                        y_out = np.clip(y, -1.0, 1.0) * 32767.0
                        y_out = y_out.astype(np.int16).tobytes()

                        processed_bytes = bytes(y_out)
                    except Exception as e:
                        self.get_logger().warning(
                            f"Failed to run processing pipeline: {e}"
                        )
                        processed_bytes = ros_audio
                else:
                    processed_bytes = ros_audio
                self.publisher_.publish(AudioData(data=processed_bytes))

                if self.debug:
                    run_frames.append(local_audio)

        except KeyboardInterrupt:
            self.get_logger().info("Stopping on user interrupt.")
        finally:
            stream.stop_stream()
            stream.close()
            self.p.terminate()
            self.get_logger().info("Audio stream closed.")

    def save_audio(self):
        if not self.debug:
            return

        self.get_logger().info("Saving audio stream.")
        output_file = os.path.join(SAVE_PATH, "last_run_audio.wav")

        import wave

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
