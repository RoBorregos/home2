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

        self.declare_parameter("OUTPUT_TOPIC", "/processedAudioChunk")
        self.declare_parameter("MIC_DEVICE_NAME", "default")
        self.declare_parameter("MIC_INPUT_CHANNELS", 6)
        self.declare_parameter("MIC_OUT_CHANNELS", 6)
        self.declare_parameter("RESPEAKER_JOIN_METHOD", "avg")
        self.declare_parameter("APPLY_PROCESSING", True)
        self.declare_parameter(
            "PIPELINE",
            ["reduce_noise", "regulate_gain", "compress", "agc", "dereverb"],
        )
        self.declare_parameter("DEBUG_DIR", "/tmp/speech_audio")

        output_topic = (
            self.get_parameter("OUTPUT_TOPIC").get_parameter_value().string_value
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
            self.get_parameter("APPLY_PROCESSING").get_parameter_value().bool_value
        )
        self.pipeline = (
            self.get_parameter("PIPELINE").get_parameter_value().string_array_value
        )
        self.debug_dir = (
            self.get_parameter("DEBUG_DIR").get_parameter_value().string_value
        )
        os.makedirs(self.debug_dir, exist_ok=True)

        self.publisher_ = self.create_publisher(AudioData, output_topic, 20)

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

        # channel extraction: which mic channel to extract when using ReSpeaker
        self.declare_parameter("EXTRACT_CHANNEL", 0)
        self.extract_channel = (
            self.get_parameter("EXTRACT_CHANNEL").get_parameter_value().integer_value
        )

        # Topic to receive the selected active mic (published by a DOA/wakeword handler)
        self.declare_parameter("ACTIVE_MIC_TOPIC", "/speech/active_mic")
        active_mic_topic = (
            self.get_parameter("ACTIVE_MIC_TOPIC").get_parameter_value().string_value
        )
        # Subscribe to active mic commands (simple JSON string or 'channel:<n>')
        from std_msgs.msg import String

        self.create_subscription(String, active_mic_topic, self._active_mic_cb, 10)

        self.get_logger().info("AudioCapturer node initialized.")

    def record(self):
        self.get_logger().info("AudioCapturer node recording.")
        CHUNK_SIZE = 512
        self.FORMAT = pyaudio.paInt16  # Signed 2 bytes.
        self.debug = False
        CHANNELS = self.mic_input_channels  # Use configured channel count.
        self.RATE = 16000
        # Use configured extract channel (dynamic, can be updated via ACTIVE_MIC_TOPIC)
        EXTRACT_CHANNEL = self.extract_channel

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
                    # Handle multi-channel ReSpeaker (e.g., 6 channels: 4 mics + 2 refs) with optional per-mic publishing and mixing strategies.
                    arr = np.frombuffer(in_data, dtype=np.int16)
                    channels = (
                        self.mic_input_channels
                        if self.mic_input_channels and self.mic_input_channels > 0
                        else CHANNELS
                    )

                    if channels <= 1 or arr.size < channels:
                        # Mono fallback: extract single channel by striding (stride=6 for typical ReSpeaker layout).
                        mono = arr[EXTRACT_CHANNEL::6] if arr.size >= 6 else arr
                        local_audio = mono.tobytes()
                    else:
                        n_frames = arr.size // channels
                        usable = arr[: n_frames * channels]
                        frames = usable.reshape(n_frames, channels)

                        # If extract channel is set, prefer extracting that single mic instead of mixing
                        if (
                            EXTRACT_CHANNEL is not None
                            and EXTRACT_CHANNEL >= 0
                            and EXTRACT_CHANNEL < frames.shape[1]
                        ):
                            extracted = frames[:, EXTRACT_CHANNEL].astype(np.int16)
                            local_audio = extracted.tobytes()
                        else:
                            # Split channels for mixing.
                            per_mics = [
                                frames[:, i].astype(np.int32)
                                for i in range(min(channels, frames.shape[1]))
                            ]

                            # Mix to mono: "sum" (improves SNR, risks clipping), "first" (fastest), or "avg" (default, balanced).
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
