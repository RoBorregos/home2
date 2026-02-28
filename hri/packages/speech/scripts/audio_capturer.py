#!/usr/bin/env python3

import os
import wave

import numpy as np
import scipy.signal
from typing import Optional
import pyaudio
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from speech.speech_api_utils import SpeechApiUtils

from frida_interfaces.msg import AudioData

# Try to import DeepFilterNet (deepfilternet). If available, use it for enhancement.
DF_AVAILABLE = False
DF_ENHANCER = None
try:
    import deepfilternet as df

    # Prefer a simple functional API if available
    if hasattr(df, "enhance"):
        DF_AVAILABLE = True
        DF_ENHANCER = df
    elif hasattr(df, "DeepFilterNet"):
        # instantiate if the class is provided
        try:
            DF_ENHANCER = df.DeepFilterNet()
            DF_AVAILABLE = True
        except Exception:
            DF_AVAILABLE = False
    else:
        DF_AVAILABLE = False
except Exception:
    DF_AVAILABLE = False
SAVE_PATH = "/workspace/src/hri/packages/speech/debug/"
run_frames = []
SAVE_IT = 100


def reduce_noise(
    y: np.ndarray,
    sr: int,
    noise_clip: Optional[np.ndarray] = None,
    n_fft: int = 2048,
    hop_length: int = 512,
    n_std_thresh: float = 3.0,
    prop_decrease: float = 1.0,
    min_gain: float = 0.95,
) -> np.ndarray:
    """Chunk-safe noise reduction for streaming.
    If chunk is too short for STFT, return audio unchanged.
    """
    y = y.astype(np.float32)
    if len(y) == 0:
        return y

    # If DeepFilterNet is available, use it for higher-quality, low-latency
    # enhancement. It handles small chunks (20ms+) reliably.
    if DF_AVAILABLE:
        try:
            # DeepFilterNet typically expects float32 or int16 PCM arrays.
            # Convert to float32 in [-1,1] range for safety.
            max_int16 = 32768.0
            y_in = (y / max_int16).astype(np.float32)

            # If DF_ENHANCER is the module with an enhance function
            if hasattr(DF_ENHANCER, "enhance"):
                y_out = DF_ENHANCER.enhance(y_in, sr)
            else:
                # If it's an instantiated object with enhance method
                y_out = DF_ENHANCER.enhance(y_in, sr)

            # Expect output in [-1,1] float32 — convert back to int16-like range
            y_out = np.nan_to_num(y_out, nan=0.0, posinf=0.0, neginf=0.0)
            y_out = np.clip(y_out, -1.0, 1.0) * max_int16
            return y_out.astype(np.float32)
        except Exception:
            # If DFN fails, fall back to spectral gating below
            pass

    # For streaming chunks (e.g., 512 samples), avoid STFT artifacts.
    # Return original audio if the chunk is too short for reliable FFT.
    if len(y) < max(256, hop_length * 2):
        return y

    # Normalize input
    max_val = np.max(np.abs(y)) + 1e-8
    y_norm = y / max_val

    n_fft_eff = min(n_fft, len(y_norm))
    if n_fft_eff <= 1:
        return y
    hop_eff = max(1, min(hop_length, n_fft_eff - 1))

    if noise_clip is None:
        n_noise = min(len(y_norm), int(0.5 * sr))
        if n_noise <= 0:
            return y
        noise_clip = y_norm[:n_noise]
    else:
        noise_clip = noise_clip.astype(np.float32) / (np.max(np.abs(noise_clip)) + 1e-8)

    try:
        _, _, S = scipy.signal.stft(
            y_norm, fs=sr, nperseg=n_fft_eff, noverlap=n_fft_eff - hop_eff
        )
        _, _, N = scipy.signal.stft(
            noise_clip, fs=sr, nperseg=n_fft_eff, noverlap=n_fft_eff - hop_eff
        )

        S_mag = np.abs(S)
        N_mag = np.abs(N)

        noise_mean = np.mean(N_mag, axis=1, keepdims=True)
        noise_std = np.std(N_mag, axis=1, keepdims=True) + 1e-8

        thresh = noise_mean + n_std_thresh * noise_std
        mask_gain = 1.0 - prop_decrease * np.minimum(
            1.0, np.maximum(0.0, (thresh - S_mag) / (S_mag + 1e-12))
        )
        # Preserve signal; never attenuate too much (configurable)
        mask_gain = np.clip(mask_gain, min_gain, 1.0)

        S_filtered = S * mask_gain
        _, y_out = scipy.signal.istft(
            S_filtered, fs=sr, nperseg=n_fft_eff, noverlap=n_fft_eff - hop_eff
        )

        if len(y_out) > len(y_norm):
            y_out = y_out[: len(y_norm)]
        elif len(y_out) < len(y_norm):
            y_out = np.pad(y_out, (0, len(y_norm) - len(y_out)))

        y_out = y_out * max_val
        y_out = np.nan_to_num(y_out, nan=0.0, posinf=0.0, neginf=0.0)
        y_out = np.clip(y_out, -32768, 32767)
        return y_out.astype(np.float32)

    except Exception:
        return y


class AudioCapturer(Node):
    def __init__(self):
        super().__init__("audio_capturer")

        self.declare_parameter("publish_topic", "/rawAudioChunk")
        self.declare_parameter("MIC_DEVICE_NAME", "default")
        self.declare_parameter("MIC_INPUT_CHANNELS", 32)
        self.declare_parameter("MIC_OUT_CHANNELS", 32)
        self.declare_parameter("ENABLE_ANC", False)

        publish_topic = (
            self.get_parameter("publish_topic").get_parameter_value().string_value
        )

        # enable active noise cancellation (spectral gating)
        self.enable_anc = (
            self.get_parameter("ENABLE_ANC").get_parameter_value().bool_value
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

        # for ANC noise clip accumulation
        self.noise_frames = []
        self.noise_clip = None

    def record(self):
        self.get_logger().info("AudioCapturer node recording.")
        iteration_step = 0
        CHUNK_SIZE = 512
        self.FORMAT = pyaudio.paInt16  # Signed 2 bytes.
        self.debug = False
        CHANNELS = 6 if self.use_respeaker else 1
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
                    in_data = np.frombuffer(in_data, dtype=np.int16)[EXTRACT_CHANNEL::6]
                    in_data = in_data.tobytes()

                local_audio = bytes(in_data)
                # Optionally apply active noise cancellation (spectral gating)
                ros_audio = bytes(local_audio)
                if self.enable_anc:
                    try:
                        # Convert to int16 numpy array
                        audio_arr = np.frombuffer(local_audio, dtype=np.int16)

                        # If we don't have a noise clip yet, accumulate initial frames
                        noise_target = int(0.5 * self.RATE)
                        if self.noise_clip is None:
                            self.noise_frames.append(audio_arr)
                            total = sum(f.shape[0] for f in self.noise_frames)
                            if total >= noise_target:
                                concat = np.concatenate(self.noise_frames)
                                self.noise_clip = concat[:noise_target].astype(
                                    np.float32
                                )
                                # free frames list
                                self.noise_frames = []
                                self.get_logger().info("ANC: noise clip captured")
                        # If we have noise clip, run reduction on this chunk
                        if self.noise_clip is not None:
                            processed = reduce_noise(
                                audio_arr.astype(np.float32),
                                sr=self.RATE,
                                noise_clip=self.noise_clip,
                            )
                            # convert back to int16 and bytes
                            processed_int16 = np.clip(processed, -32768, 32767).astype(
                                np.int16
                            )
                            ros_audio = processed_int16.tobytes()
                    except Exception as e:
                        self.get_logger().warn(f"ANC processing error: {e}")
                self.publisher_.publish(AudioData(data=ros_audio))

                if self.debug:
                    run_frames.append(local_audio)

                iteration_step += 1
                if iteration_step % SAVE_IT == 0:
                    iteration_step = 0
                    self.save_audio()

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
