#!/usr/bin/env python3

import os
import queue
import threading
import time
import wave

import numpy as np
import rclpy
import scipy.signal
import torch
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from frida_interfaces.msg import AudioData
import df as DF_MODULE

SAVE_PATH = "/tmp"
SAVE_IT = 100
run_frames = []

QUEUE_SIZE = 20
SAMPLE_RATE = 16000
RESAMPLE_FACTOR = 3  # 16kHz * 3 = 48kHz (DeepFilterNet native rate)
INT16_SCALE = 32768.0
INT16_MIN = -32768
INT16_MAX = 32767
# Auto-disable DF after this many consecutive frames slower than real-time
_DF_MAX_SLOW_CHUNKS = 20


class NoiseCancellation(Node):
    def __init__(self):
        super().__init__("noise_cancellation")

        self.declare_parameter("RAW_AUDIO_TOPIC", "/hri/rawAudioChunk")
        self.declare_parameter("PROCESSED_AUDIO_TOPIC", "/hri/processedAudioChunk")
        self.declare_parameter("ENABLE_ANC", True)
        self.declare_parameter("GAIN", 1.0)
        self.declare_parameter("DEBUG", False)
        self.declare_parameter("DF_MODEL_PATH", "../assets/downloads")

        input_topic = self.get_parameter("RAW_AUDIO_TOPIC").value
        output_topic = self.get_parameter("PROCESSED_AUDIO_TOPIC").value
        self.enable_anc = self.get_parameter("ENABLE_ANC").value
        self.gain = self.get_parameter("GAIN").value
        self.debug = self.get_parameter("DEBUG").value

        # Resolve model path relative to this script's package root
        script_dir = os.path.dirname(os.path.realpath(__file__))
        # Path from script is ../../assets/downloads
        base_path = os.path.abspath(os.path.join(script_dir, "..", ".."))
        self.df_model_path = os.path.join(
            base_path, self.get_parameter("DF_MODEL_PATH").value
        )

        self.df_model = None
        self.df_state = None
        self.df_ready = False
        self.use_df = self.enable_anc
        self._consecutive_slow_chunks = 0

        self.publisher_ = self.create_publisher(AudioData, output_topic, QUEUE_SIZE)
        self.create_subscription(
            AudioData, input_topic, self._audio_callback, QUEUE_SIZE
        )

        # Queue + background thread so DF inference never blocks the ROS2 executor
        self._audio_queue: queue.Queue = queue.Queue(maxsize=QUEUE_SIZE)
        threading.Thread(target=self._process_loop, daemon=True).start()

        self.get_logger().info(
            f"NoiseCancellation node ready. {input_topic} → {output_topic}"
        )

        if self.use_df:
            self.get_logger().info(
                "Starting DeepFilterNet in background thread. ANC will be active once ready..."
            )
            threading.Thread(target=self._init_df_async, daemon=True).start()
        else:
            self.get_logger().info(
                "ENABLE_ANC=False or DeepFilterNet not available. Passing audio through."
            )

    def _init_df_async(self):
        try:
            model_dir = os.path.join(self.df_model_path, "DeepFilterNet3")
            if not os.path.isdir(model_dir):
                self.get_logger().error(
                    f"DeepFilterNet model not found at {model_dir}. "
                    "Run docker/hri/scripts/download-model.sh on a machine with "
                    "internet access and copy the 'assets' folder to hri/packages/speech/."
                )
                self.use_df = False
                return

            self.get_logger().info(f"Loading DeepFilterNet model from {model_dir}")
            self.df_model, self.df_state, _ = DF_MODULE.init_df(
                model_base_dir=model_dir
            )
            if torch.cuda.is_available():
                self.df_model = self.df_model.to("cuda")
                self.get_logger().info("CUDA enabled for noise suppression.")
            else:
                self.get_logger().warn(
                    "CUDA not available — DeepFilterNet will run on CPU. "
                    "ANC will be auto-disabled if CPU cannot keep up with real-time audio. "
                    "Add 'runtime: nvidia' to docker/hri/compose/hri-ros.yaml for GPU ANC."
                )

            # Pre-warm: run dummy inferences so the first real chunk is not slow.
            # Input tensor must stay on CPU — DF_MODULE.enhance handles device placement
            # internally and will fail with 'can't convert cuda tensor to numpy' if
            # the input is already on CUDA.
            self.get_logger().info("Pre-warming DeepFilterNet (3 dummy inferences)...")
            dummy = (np.random.randn(1024 * RESAMPLE_FACTOR) * 0.01).astype(np.float32)
            dummy_tensor = torch.from_numpy(dummy).unsqueeze(0)  # keep on CPU
            for _ in range(3):
                with torch.no_grad():
                    DF_MODULE.enhance(self.df_model, self.df_state, dummy_tensor)

            self.df_ready = True
            self.get_logger().info("DeepFilterNet ready. Noise suppression active.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize DeepFilterNet: {e}")
            self.use_df = False

    def _audio_callback(self, msg):
        # Non-blocking: drop oldest chunk if queue is full to avoid stalling the executor
        try:
            self._audio_queue.put_nowait(msg)
        except queue.Full:
            try:
                self._audio_queue.get_nowait()  # drop oldest
            except queue.Empty:
                pass
            self._audio_queue.put_nowait(msg)

    def _process_loop(self):
        """Background thread: pulls from queue, runs DF inference, publishes."""
        iteration_step = 0
        while True:
            msg = self._audio_queue.get()
            audio_arr = np.frombuffer(msg.data, dtype=np.int16)

            if self.use_df and self.df_ready:
                chunk_duration_s = len(audio_arr) / SAMPLE_RATE
                t0 = time.monotonic()
                audio_arr = self._enhance(audio_arr)
                elapsed = time.monotonic() - t0

                if elapsed > chunk_duration_s:
                    self._consecutive_slow_chunks += 1
                    if self._consecutive_slow_chunks >= _DF_MAX_SLOW_CHUNKS:
                        self.get_logger().warn(
                            f"DeepFilterNet took {elapsed * 1000:.0f}ms for a "
                            f"{chunk_duration_s * 1000:.0f}ms chunk "
                            f"({_DF_MAX_SLOW_CHUNKS} consecutive slow frames). "
                            "Disabling ANC to prevent audio gaps. "
                            "Add 'runtime: nvidia' to hri-ros.yaml for real-time GPU ANC."
                        )
                        self.use_df = False
                        self._consecutive_slow_chunks = 0
                else:
                    self._consecutive_slow_chunks = 0

            elif self.gain != 1.0:
                audio_arr = np.clip(
                    audio_arr.astype(np.float32) * self.gain, INT16_MIN, INT16_MAX
                ).astype(np.int16)

            out_bytes = audio_arr.tobytes()
            self.publisher_.publish(AudioData(data=out_bytes))

            if self.debug:
                run_frames.append(out_bytes)
                iteration_step += 1
                if iteration_step % SAVE_IT == 0:
                    iteration_step = 0
                    self.save_audio()

    def save_audio(self):
        self.get_logger().info("Saving processed audio stream.")
        output_file = os.path.join(SAVE_PATH, "last_run_noise_cancelled.wav")
        with wave.open(output_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # int16 = 2 bytes
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(b"".join(run_frames))

    def _enhance(self, audio_int16):
        """Process audio: 16k -> 48k -> DeepFilterNet -> 16k -> Gain

        Normalises by the chunk's actual peak before DF (not fixed INT16_SCALE)
        so DF always receives a full-scale [-1, 1] signal regardless of mic level.
        Dividing by INT16_SCALE when the mic only uses 5% of the range passes
        amplitude ~0.05 to DF, which classifies speech as noise and suppresses it.
        """
        try:
            # 1. Convert to float32 and normalise by actual peak
            audio_float = audio_int16.astype(np.float32)
            peak = float(np.max(np.abs(audio_float)))
            if peak == 0:
                return audio_int16
            audio_float = audio_float / peak  # always in [-1, 1]

            # 2. Resample 16k -> 48k; force float32 (scipy may return float64)
            audio_48k = scipy.signal.resample_poly(
                audio_float, RESAMPLE_FACTOR, 1
            ).astype(np.float32)

            # 3. Prepare tensor — keep on CPU, shape [1, T].
            # DF_MODULE.enhance moves data to the model's device internally.
            tensor = torch.from_numpy(audio_48k).unsqueeze(0)

            # 4. Neural noise suppression
            with torch.no_grad():
                enhanced_48k_t = DF_MODULE.enhance(self.df_model, self.df_state, tensor)

            enhanced_48k = enhanced_48k_t.cpu().numpy().squeeze().astype(np.float32)

            # 5. Resample 48k -> 16k; force float32
            enhanced_16k = scipy.signal.resample_poly(
                enhanced_48k, 1, RESAMPLE_FACTOR
            ).astype(np.float32)

            # 6. Fallback if DF produced NaN (can happen on cold GPU frames)
            if np.isnan(enhanced_16k).any():
                self.get_logger().warn(
                    "DF produced NaN — passing through original audio."
                )
                return audio_int16

            # 7. Scale back to original int16 range and apply gain
            return np.clip(
                enhanced_16k * peak * self.gain, INT16_MIN, INT16_MAX
            ).astype(np.int16)
        except Exception as e:
            self.get_logger().error(f"Error enhancing audio frame: {e}")
            return audio_int16


def main(args=None):
    rclpy.init(args=args)
    node = NoiseCancellation()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
