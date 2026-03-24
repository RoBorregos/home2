#!/usr/bin/env python3

"""
Voice Activity Detection (VAD) node.

Sits between noise_cancellation and hear_streaming in the audio pipeline.
Subscribes to processed audio, applies energy + autocorrelation VAD with
adaptive speaker identity locking (MFCC cosine similarity), and forwards
audio chunks only while the locked speaker is active (plus a silence tail
to let the STT finalize the last utterance).

Topics:
  Subscribes : PROCESSED_AUDIO_TOPIC  (/hri/processedAudioChunk)
  Publishes  : VAD_AUDIO_TOPIC        (/hri/vadAudioChunk)
               VOICE_ACTIVITY_TOPIC   (/hri/voice_activity)  Bool
"""

import librosa
import numpy as np
from scipy.spatial.distance import cosine as cosine_distance
import rclpy
from frida_interfaces.msg import AudioData
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

# Fundamental frequency range for human voice (Hz)
_VOWEL_FREQ_LOW = 85
_VOWEL_FREQ_HIGH = 255


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

        # Pre-compute lag bounds for autocorrelation pitch detection
        self._low_lag = int(self.sample_rate / _VOWEL_FREQ_HIGH)
        self._high_lag = int(self.sample_rate / _VOWEL_FREQ_LOW)

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

    # ------------------------------------------------------------------
    def _get_mfcc(self, audio_f: np.ndarray) -> np.ndarray:
        """Extract mean MFCCs — timbre fingerprint of the speaker."""
        mfccs = librosa.feature.mfcc(y=audio_f, sr=self.sample_rate, n_mfcc=13)
        return np.mean(mfccs, axis=1)

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
        self.get_logger().info("Speaker lock released")

    # ------------------------------------------------------------------
    def _audio_cb(self, msg: AudioData) -> None:
        audio = np.frombuffer(bytes(msg.data), dtype=np.int16)
        chunk_duration = len(audio) / self.sample_rate

        if self._is_speech(audio):
            self._silence_counter = 0.0
            if not self._speech_active:
                self._speech_active = True
                self._activity_pub.publish(Bool(data=True))
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
                    self.get_logger().info("Voice activity: speaker ended talking")
                    self._reset_speaker_lock()


# ---------------------------------------------------------------------------
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
