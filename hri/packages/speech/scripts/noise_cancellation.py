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

SAVE_PATH = "/workspace/src/hri/packages/speech/debug/audios"
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


# ── pyaec (SpeexDSP AEC) ─────────────────────────────────────────────────────

_AEC_FRAME_SIZE = 160  # 10ms at 16kHz — SpeexDSP requirement
_AEC_FILTER_LEN = 3200  # 200ms echo tail at 16kHz

try:
    from pyaec import Aec as _PyAec

    _HAS_PYAEC = True
except ImportError:
    _HAS_PYAEC = False


class ReferenceRingBuffer:
    """
    Thread-safe ring buffer that accumulates reference (speaker) samples.

    The reference signal arrives in chunks from say.py at real-time pace.
    The mic signal also arrives in chunks. These two streams are NOT
    synchronized — chunks may arrive at slightly different times and sizes.

    This buffer accumulates reference samples so that when a mic chunk needs
    processing, we can always pull the exact number of reference samples needed.
    If the speaker is not playing, the buffer returns zeros (silence).
    """

    def __init__(self, capacity: int = SAMPLE_RATE * 2):
        self._buf = np.zeros(capacity, dtype=np.int16)
        self._write_pos = 0
        self._read_pos = 0
        self._capacity = capacity
        self._lock = threading.Lock()
        self._has_data = False  # True once any reference has been written

    def write(self, samples: np.ndarray):
        """Append reference samples to the buffer."""
        with self._lock:
            n = len(samples)
            if n >= self._capacity:
                # If writing more than capacity, keep only the last chunk
                self._buf[:] = samples[-self._capacity :]
                self._write_pos = 0
                self._read_pos = 0
                self._has_data = True
                return

            end = self._write_pos + n
            if end <= self._capacity:
                self._buf[self._write_pos : end] = samples
            else:
                first = self._capacity - self._write_pos
                self._buf[self._write_pos :] = samples[:first]
                self._buf[: n - first] = samples[first:]

            self._write_pos = end % self._capacity
            self._has_data = True

    def read(self, n: int) -> np.ndarray:
        """
        Read n reference samples aligned with the mic stream.

        If reference data is available, returns it and advances the read pointer.
        If no reference has been written (speaker silent), returns zeros.
        """
        with self._lock:
            if not self._has_data:
                return np.zeros(n, dtype=np.int16)

            available = (self._write_pos - self._read_pos) % self._capacity
            if available <= 0 and self._write_pos == self._read_pos:
                # Buffer might be full or empty — check has_data
                available = 0

            if available < n:
                # Not enough reference samples — pad with zeros
                out = np.zeros(n, dtype=np.int16)
                if available > 0:
                    end = self._read_pos + available
                    if end <= self._capacity:
                        out[:available] = self._buf[self._read_pos : end]
                    else:
                        first = self._capacity - self._read_pos
                        out[:first] = self._buf[self._read_pos :]
                        out[first:available] = self._buf[: available - first]
                self._read_pos = self._write_pos
                return out

            # Read n samples
            out = np.zeros(n, dtype=np.int16)
            end = self._read_pos + n
            if end <= self._capacity:
                out[:] = self._buf[self._read_pos : end]
            else:
                first = self._capacity - self._read_pos
                out[:first] = self._buf[self._read_pos :]
                out[first:] = self._buf[: n - first]

            self._read_pos = end % self._capacity
            return out

    @property
    def active(self) -> bool:
        """True if reference data has been written recently."""
        return self._has_data


class SpeexAec:
    """
    SpeexDSP-based AEC for real-time echo cancellation.

    Processes audio in 10ms frames (160 samples at 16kHz).
    Accepts any input chunk size — internally splits into 10ms frames.
    """

    def __init__(self):
        if not _HAS_PYAEC:
            raise ImportError("pyaec not installed. Install: pip install pyaec")
        self._aec = _PyAec(_AEC_FRAME_SIZE, _AEC_FILTER_LEN, SAMPLE_RATE, True)

    def process_block(self, mic_block: np.ndarray, ref_block: np.ndarray) -> np.ndarray:
        """Process a chunk through SpeexDSP AEC. Returns echo-cancelled int16."""
        n = min(len(mic_block), len(ref_block))
        mic_i16 = np.clip(mic_block[:n], INT16_MIN, INT16_MAX).astype(np.int16)
        ref_i16 = np.clip(ref_block[:n], INT16_MIN, INT16_MAX).astype(np.int16)

        output = np.zeros(n, dtype=np.int16)
        fs = _AEC_FRAME_SIZE

        n_frames = n // fs
        for i in range(n_frames):
            s = i * fs
            e = s + fs
            out = self._aec.cancel_echo(
                mic_i16[s:e].tolist(),
                ref_i16[s:e].tolist(),
            )
            output[s:e] = np.array(out, dtype=np.int16)

        # Handle remaining samples (< 160) by padding
        remainder = n - n_frames * fs
        if remainder > 0:
            s = n_frames * fs
            mic_padded = np.zeros(fs, dtype=np.int16)
            ref_padded = np.zeros(fs, dtype=np.int16)
            mic_padded[:remainder] = mic_i16[s : s + remainder]
            ref_padded[:remainder] = ref_i16[s : s + remainder]
            out = self._aec.cancel_echo(mic_padded.tolist(), ref_padded.tolist())
            output[s : s + remainder] = np.array(out[:remainder], dtype=np.int16)

        return output


class NoiseCancellation(Node):
    def __init__(self):
        super().__init__("noise_cancellation")

        self.declare_parameter("RAW_AUDIO_TOPIC", "/hri/rawAudioChunk")
        self.declare_parameter("PROCESSED_AUDIO_TOPIC", "/hri/processedAudioChunk")
        self.declare_parameter("REFERENCE_AUDIO_TOPIC", "/hri/referenceAudioChunk")
        self.declare_parameter("ENABLE_ANC", True)
        self.declare_parameter("ENABLE_AEC", False)
        self.declare_parameter("GAIN", 1.0)
        self.declare_parameter("DEBUG", False)
        self.declare_parameter("DF_MODEL_PATH", "assets/downloads")
        self.declare_parameter("DF_ATTEN_LIM_DB", 12.0)

        input_topic = self.get_parameter("RAW_AUDIO_TOPIC").value
        output_topic = self.get_parameter("PROCESSED_AUDIO_TOPIC").value
        ref_topic = self.get_parameter("REFERENCE_AUDIO_TOPIC").value
        self.enable_anc = self.get_parameter("ENABLE_ANC").value
        self.enable_aec = self.get_parameter("ENABLE_AEC").value
        self.gain = self.get_parameter("GAIN").value
        self.debug = self.get_parameter("DEBUG").value
        self.df_atten_lim_db = self.get_parameter("DF_ATTEN_LIM_DB").value

        # Resolve model path relative to this script's package root
        script_dir = os.path.dirname(os.path.realpath(__file__))
        base_path = os.path.abspath(os.path.join(script_dir, ".."))
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

        # ── AEC (SpeexDSP via pyaec) setup ──────────────────────────────
        self._aec: SpeexAec | None = None
        self._ref_ring = ReferenceRingBuffer(capacity=SAMPLE_RATE * 2)  # 2s buffer

        if self.enable_aec:
            if _HAS_PYAEC:
                self._aec = SpeexAec()
                self.create_subscription(
                    AudioData, ref_topic, self._ref_callback, QUEUE_SIZE
                )
                self.get_logger().info(
                    f"AEC enabled (SpeexDSP via pyaec). Reference: {ref_topic}"
                )
            else:
                self.get_logger().warn(
                    "ENABLE_AEC=True but pyaec not installed. "
                    "Install: pip install pyaec. AEC disabled."
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

            self.get_logger().info("Pre-warming DeepFilterNet (3 dummy inferences)...")
            dummy = (np.random.randn(1024 * RESAMPLE_FACTOR) * 0.01).astype(np.float32)
            dummy_tensor = torch.from_numpy(dummy).unsqueeze(0)
            for _ in range(3):
                with torch.no_grad():
                    DF_MODULE.enhance(self.df_model, self.df_state, dummy_tensor)

            self.df_ready = True
            self.get_logger().info("DeepFilterNet ready. Noise suppression active.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize DeepFilterNet: {e}")
            self.use_df = False

    def _ref_callback(self, msg):
        """Accumulate reference samples into the ring buffer."""
        ref_arr = np.frombuffer(msg.data, dtype=np.int16)
        self._ref_ring.write(ref_arr)

    def _audio_callback(self, msg):
        try:
            self._audio_queue.put_nowait(msg)
        except queue.Full:
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                pass
            self._audio_queue.put_nowait(msg)

    def _process_loop(self):
        """Background thread: AEC → DeepFilterNet → Gain → publish."""
        iteration_step = 0
        while True:
            msg = self._audio_queue.get()
            audio_arr = np.frombuffer(msg.data, dtype=np.int16)

            # ── AEC: cancel speaker echo ─────────────────────────────
            if self._aec is not None:
                # Pull matching number of reference samples from ring buffer
                ref_arr = self._ref_ring.read(len(audio_arr))
                audio_arr = self._aec.process_block(audio_arr, ref_arr)

            # ── DeepFilterNet noise suppression ──────────────────────
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
        os.makedirs(SAVE_PATH, exist_ok=True)
        output_file = os.path.join(SAVE_PATH, "last_run_noise_cancelled.wav")
        with wave.open(output_file, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(b"".join(run_frames))

    def _enhance(self, audio_int16):
        """Process audio: 16k -> 48k -> DeepFilterNet -> 16k -> Gain"""
        try:
            audio_float = audio_int16.astype(np.float32)
            peak = float(np.max(np.abs(audio_float)))
            if peak == 0:
                return audio_int16
            audio_float = audio_float / peak

            audio_48k = scipy.signal.resample_poly(
                audio_float, RESAMPLE_FACTOR, 1
            ).astype(np.float32)

            tensor = torch.from_numpy(audio_48k).unsqueeze(0)

            with torch.no_grad():
                enhanced_48k_t = DF_MODULE.enhance(
                    self.df_model,
                    self.df_state,
                    tensor,
                    atten_lim_db=self.df_atten_lim_db,
                )

            enhanced_48k = enhanced_48k_t.cpu().numpy().squeeze().astype(np.float32)

            enhanced_16k = scipy.signal.resample_poly(
                enhanced_48k, 1, RESAMPLE_FACTOR
            ).astype(np.float32)

            if np.isnan(enhanced_16k).any():
                self.get_logger().warn(
                    "DF produced NaN — passing through original audio."
                )
                return audio_int16

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
