#!/usr/bin/env python3

import os
import wave
import json

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

        # Parameters
        self.declare_parameter("OUTPUT_TOPIC", "/processedAudioChunk")
        self.declare_parameter("MIC_DEVICE_NAME", "default")
        self.declare_parameter("MIC_INPUT_CHANNELS", 1)
        self.declare_parameter("MIC_OUT_CHANNELS", 1)
        self.declare_parameter("RESPEAKER_JOIN_METHOD", "avg")
        self.declare_parameter("APPLY_PROCESSING", True)
        try:
            self.declare_parameter("PIPELINE", [])
        except Exception as e:
            self.get_logger().warning(f"Could not declare PIPELINE param: {e}")
        self.declare_parameter("DEBUG_DIR", "/tmp/speech_audio")
        self.declare_parameter("LOG_RMS_EVERY", 0)

        self.output_topic = self.get_parameter("OUTPUT_TOPIC").get_parameter_value().string_value
        self.use_respeaker = SpeechApiUtils.respeaker_available()
        self.get_logger().info(f"ReSpeaker detected: {self.use_respeaker}")

        self.mic_device_name = self.get_parameter("MIC_DEVICE_NAME").get_parameter_value().string_value
        self.mic_input_channels = self.get_parameter("MIC_INPUT_CHANNELS").get_parameter_value().integer_value
        self.mic_out_channels = self.get_parameter("MIC_OUT_CHANNELS").get_parameter_value().integer_value
        self.join_method = self.get_parameter("RESPEAKER_JOIN_METHOD").get_parameter_value().string_value
        self.apply_processing = self.get_parameter("APPLY_PROCESSING").get_parameter_value().bool_value
        self.debug_dir = self.get_parameter("DEBUG_DIR").get_parameter_value().string_value
        self.log_rms_every = int(self.get_parameter("LOG_RMS_EVERY").get_parameter_value().integer_value)
        os.makedirs(self.debug_dir, exist_ok=True)

        # Read PIPELINE tolerant to several representations
        try:
            pv = self.get_parameter("PIPELINE").get_parameter_value()
            try:
                self.pipeline = list(pv.string_array_value) if pv.string_array_value else []
            except Exception:
                try:
                    b = pv.byte_array_value
                    if isinstance(b, (bytes, bytearray)):
                        try:
                            self.pipeline = json.loads(b.decode("utf-8"))
                        except Exception:
                            self.pipeline = []
                    else:
                        self.pipeline = [x.decode("utf-8") if isinstance(x, (bytes, bytearray)) else str(x) for x in b]
                except Exception:
                    self.pipeline = []
        except Exception:
            self.pipeline = []

        if self.apply_processing and (not self.pipeline):
            self.pipeline = ["reduce_noise"]
            self.get_logger().info("No PIPELINE provided; defaulting to ['reduce_noise'] for safe testing.")

        self.publisher_ = self.create_publisher(AudioData, self.output_topic, 20)

        # Audio stream defaults
        self.FORMAT = pyaudio.paInt16
        self.RATE = 16000
        self.CHUNK_SIZE = 1024
        self.EXTRACT_CHANNEL = 0

        self.input_device_index = SpeechApiUtils.getIndexByNameAndChannels(
            self.mic_device_name, self.mic_input_channels, self.mic_out_channels
        )
        self.get_logger().info("Input device index: %s" % str(self.input_device_index))

        self.debug = False

    def record(self):
        self.p = pyaudio.PyAudio()
        channels = max(1, self.mic_input_channels)
        try:
            stream = self.p.open(
                input_device_index=self.input_device_index,
                format=self.FORMAT,
                channels=channels,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK_SIZE,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {e}")
            raise

        iteration_step = 0
        try:
            while rclpy.ok():
                in_data = stream.read(self.CHUNK_SIZE, exception_on_overflow=False)

                if self.use_respeaker and self.mic_input_channels > 1:
                    arr = np.frombuffer(in_data, dtype=np.int16)
                    channels = self.mic_input_channels
                    if arr.size < channels:
                        mono = arr
                    else:
                        n_frames = arr.size // channels
                        frames = arr[: n_frames * channels].reshape(n_frames, channels)
                        per_mics = [frames[:, i].astype(np.int32) for i in range(min(channels, frames.shape[1]))]
                        if self.join_method == "sum":
                            mixed = np.sum(np.vstack(per_mics), axis=0)
                            mixed = np.clip(mixed, -32768, 32767).astype(np.int16)
                        elif self.join_method == "first":
                            mixed = per_mics[0].astype(np.int16)
                        else:
                            mixed = np.mean(np.vstack(per_mics), axis=0).astype(np.int16)
                        mono = mixed
                    local_audio = mono.tobytes()
                else:
                    local_audio = in_data

                ros_audio = bytes(local_audio)

                # Processing pipeline
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
                                    y = compress_dynamic_range(y, threshold_db=-20.0, ratio=2.0)
                                elif step == "agc":
                                    y, _ = adaptive_agc(y, target_rms=0.05)
                                elif step == "dereverb":
                                    y = dereverb_spectral(y, self.RATE)
                                else:
                                    pass
                            except Exception as e:
                                self.get_logger().warning(f"Processing step {step} failed: {e}")

                        y_out = np.clip(y, -1.0, 1.0) * 32767.0
                        y_out = y_out.astype(np.int16).tobytes()
                        processed_bytes = bytes(y_out)
                    except Exception as e:
                        self.get_logger().warning(f"Failed to run processing pipeline: {e}")
                        processed_bytes = ros_audio
                else:
                    processed_bytes = ros_audio

                # RMS logging for diagnostics
                if self.log_rms_every and (iteration_step % max(1, self.log_rms_every) == 0):
                    try:
                        arrf = np.frombuffer(ros_audio, dtype=np.int16).astype(np.float32) / 32768.0
                        pre_rms = float(np.sqrt((arrf ** 2).mean()))
                        arrf2 = np.frombuffer(processed_bytes, dtype=np.int16).astype(np.float32) / 32768.0
                        post_rms = float(np.sqrt((arrf2 ** 2).mean()))
                        self.get_logger().info(f"RMS before processing (iter {iteration_step}): {pre_rms:.6f}")
                        self.get_logger().info(f"RMS after processing  (iter {iteration_step}): {post_rms:.6f}")
                        if post_rms < 1e-5:
                            self.get_logger().warning(f"Processed RMS very low ({post_rms:.6g}) — processing may be suppressing signal.")
                    except Exception:
                        pass

                # Publish
                try:
                    self.publisher_.publish(AudioData(data=processed_bytes))
                except Exception:
                    pass

                if self.debug:
                    run_frames.append(local_audio)

                iteration_step += 1

        except KeyboardInterrupt:
            self.get_logger().info("Stopping on user interrupt.")
        finally:
            try:
                stream.stop_stream()
                stream.close()
            except Exception:
                pass
            try:
                self.p.terminate()
            except Exception:
                pass
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
