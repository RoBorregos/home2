#!/usr/bin/env python3

import numpy as np
import scipy.fft
from scipy.spatial.distance import cosine as cosine_distance
import rclpy
from frida_constants.hri_constants import VOWEL_FREQ_LOW, VOWEL_FREQ_HIGH
from frida_interfaces.msg import AudioData
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class VoiceDetection(Node):
    def __init__(self):
        super().__init__("voice_detection")

        self.declare_parameter("PROCESSED_AUDIO_TOPIC", "/hri/processedAudioChunk")
        self.declare_parameter("VAD_AUDIO_TOPIC", "/hri/vadAudioChunk")
        self.declare_parameter("VOICE_ACTIVITY_TOPIC", "/hri/voice_activity")
        self.declare_parameter("ENERGY_THRESHOLD", 1500.0)
        self.declare_parameter("CORRELATION_THRESHOLD", 0.5)
        self.declare_parameter("SIMILARITY_THRESHOLD", 0.80)
        self.declare_parameter("SILENCE_LIMIT", 1.5)
        self.declare_parameter("SAMPLE_RATE", 16000)
        self.declare_parameter("DEBUG", False)

        processed_topic = (
            self.get_parameter("PROCESSED_AUDIO_TOPIC")
            .get_parameter_value()
            .string_value
        )
        vad_topic = (
            self.get_parameter("VAD_AUDIO_TOPIC").get_parameter_value().string_value
        )
        activity_topic = (
            self.get_parameter("VOICE_ACTIVITY_TOPIC")
            .get_parameter_value()
            .string_value
        )
        self.energy_threshold = (
            self.get_parameter("ENERGY_THRESHOLD").get_parameter_value().double_value
        )
        self.correlation_threshold = (
            self.get_parameter("CORRELATION_THRESHOLD")
            .get_parameter_value()
            .double_value
        )
        self.similarity_threshold = (
            self.get_parameter("SIMILARITY_THRESHOLD")
            .get_parameter_value()
            .double_value
        )
        self.silence_limit = (
            self.get_parameter("SILENCE_LIMIT").get_parameter_value().double_value
        )
        self.sample_rate = (
            self.get_parameter("SAMPLE_RATE").get_parameter_value().integer_value
        )
        self.debug = self.get_parameter("DEBUG").get_parameter_value().bool_value

        # Pre-compute lag bounds for autocorrelation pitch detection
        self._low_lag = int(self.sample_rate / VOWEL_FREQ_HIGH)
        self._high_lag = int(self.sample_rate / VOWEL_FREQ_LOW)

        self._silence_counter = 0.0
        self._speech_active = False

        # Adaptive speaker identity lock (from AdaptiveSpeakerVAD)
        self._target_mfcc = None
        self._is_locked = False

        self._audio_pub = self.create_publisher(AudioData, vad_topic, 10)
        self._activity_pub = self.create_publisher(Bool, activity_topic, 10)
        self.create_subscription(AudioData, processed_topic, self._audio_cb, 10)

        self.get_logger().info(
            f"VoiceDetection ready  |  in: {processed_topic}  |  out: {vad_topic}"
        )

    def _get_mfcc(self, audio_f: np.ndarray) -> np.ndarray:
        """Extract mean MFCCs using numpy/scipy — no librosa/numba needed."""
        n_mfcc = 13
        n_fft = 512
        n_mels = 40
        hop = 160

        # Frame + Hann window
        num_frames = max(1, (len(audio_f) - n_fft) // hop + 1)
        frames = (
            np.stack(
                [
                    audio_f[i * hop : i * hop + n_fft] * np.hanning(n_fft)
                    for i in range(num_frames)
                    if i * hop + n_fft <= len(audio_f)
                ]
            )
            if num_frames > 0
            else np.zeros((1, n_fft))
        )

        # Power spectrum
        power = np.abs(np.fft.rfft(frames, n=n_fft)) ** 2  # (frames, n_fft//2+1)

        # Mel filterbank
        fmin, fmax = 0.0, self.sample_rate / 2.0
        mel_pts = np.linspace(
            2595 * np.log10(1 + fmin / 700),
            2595 * np.log10(1 + fmax / 700),
            n_mels + 2,
        )
        hz_pts = 700 * (10 ** (mel_pts / 2595) - 1)
        bins = np.floor((n_fft + 1) * hz_pts / self.sample_rate).astype(int)
        fb = np.zeros((n_mels, n_fft // 2 + 1))
        for m in range(1, n_mels + 1):
            lo, mid, hi = bins[m - 1], bins[m], bins[m + 1]
            if mid > lo:
                fb[m - 1, lo:mid] = (np.arange(lo, mid) - lo) / (mid - lo)
            if hi > mid:
                fb[m - 1, mid:hi] = (hi - np.arange(mid, hi)) / (hi - mid)

        mel_e = np.dot(power, fb.T)
        mel_e = np.where(mel_e == 0, np.finfo(float).eps, mel_e)
        log_mel = np.log(mel_e)

        # DCT → MFCCs
        mfccs = scipy.fft.dct(log_mel, type=2, norm="ortho", axis=-1)[:, :n_mfcc]
        return np.mean(mfccs, axis=0)

    def _is_periodic(self, audio_f: np.ndarray) -> bool:
        """Return True if audio has periodic pitch structure (voiced speech)."""
        corr = np.correlate(audio_f, audio_f, mode="full")
        corr = corr[len(corr) // 2 :]  # keep non-negative lags

        if len(corr) <= self._high_lag or corr[0] == 0:
            return False

        peak = np.max(corr[self._low_lag : self._high_lag])
        return (peak / corr[0]) > self.correlation_threshold

    def _is_speech(self, audio: np.ndarray) -> bool:
        """Return True if the chunk contains speech from the locked speaker.

        Three cascaded tests (mirrors AdaptiveSpeakerVAD.process_frame):
          1. RMS energy gate — rejects silence and background noise.
          2. Autocorrelation pitch check — rejects non-vocal noise.
          3. MFCC cosine similarity — locks onto and tracks the primary speaker.
        """
        audio_f = audio.astype(np.float32)

        # 1. Energy gate
        rms = np.sqrt(np.mean(audio_f**2))
        if rms < self.energy_threshold:
            return False

        # 2. Autocorrelation pitch check
        if not self._is_periodic(audio_f):
            return False

        # 3. Speaker identity (MFCC lock-on)
        current_mfcc = self._get_mfcc(audio_f)

        if not self._is_locked:
            # Lock onto the first strong speaker (same as AdaptiveSpeakerVAD)
            self._target_mfcc = current_mfcc
            self._is_locked = True
            if self.debug:
                self.get_logger().info("Speaker lock-on: primary speaker acquired")
            return True
        else:
            similarity = 1.0 - cosine_distance(current_mfcc, self._target_mfcc)
            if similarity > self.similarity_threshold:
                return True
            else:
                self.get_logger().debug(
                    f"Foreign voice rejected (similarity={similarity:.2f})"
                )
                return False

    def _reset_speaker_lock(self) -> None:
        """Release the speaker identity lock (mirrors AdaptiveSpeakerVAD.reset)."""
        self._target_mfcc = None
        self._is_locked = False
        if self.debug:
            self.get_logger().info("Speaker lock released")

    def _audio_cb(self, msg: AudioData) -> None:
        audio = np.frombuffer(bytes(msg.data), dtype=np.int16)
        chunk_duration = len(audio) / self.sample_rate

        if self._is_speech(audio):
            self._silence_counter = 0.0
            if not self._speech_active:
                self._speech_active = True
                self._activity_pub.publish(Bool(data=True))
                if self.debug:
                    self.get_logger().info("Voice activity: speech started")
            self._audio_pub.publish(msg)

        else:
            if self._speech_active:
                # Forward audio during the silence tail so the STT can
                # process the end of the utterance before we cut off.
                self._silence_counter += chunk_duration
                self._audio_pub.publish(msg)

                if self._silence_counter >= self.silence_limit:
                    self._speech_active = False
                    self._silence_counter = 0.0
                    self._activity_pub.publish(Bool(data=False))
                    if self.debug:
                        self.get_logger().info("Voice activity: speaker ended talking")
                    self._reset_speaker_lock()


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(VoiceDetection())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
