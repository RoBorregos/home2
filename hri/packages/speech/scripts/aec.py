#!/usr/bin/env python3
"""
Acoustic Echo Cancellation (AEC) — gate-based suppression with soft fade.

Inserts between audio_capturer and noise_cancellation:
  /hri/rawAudioChunk → [AEC] → /hri/aecAudioChunk → [NoiseCancellation]

Strategy:
  When say.py is playing audio (/saying == True), the mic input is attenuated
  so the robot's own voice is not fed into the STT/KWS pipeline (echo).
  A linear fade avoids audible clicks at gate transitions.
  A configurable post-speech hold continues suppression briefly after speech
  ends to cancel speaker reverb/tail.

  Optional barge-in detection: if the mic energy significantly exceeds the
  running average of the echo level while the robot is speaking, the audio
  is passed through so a human can interrupt the robot mid-sentence.

Why gate-based and not adaptive filtering (LMS/WebRTC)?
  Adaptive filters require the reference signal (what is being played) at the
  same sample rate as the microphone input, aligned in time. Capturing that
  reference requires OS-level audio loopback (e.g. PulseAudio monitor source),
  which is hardware-dependent and fragile in Docker/embedded deployments.
  For a turn-taking HRI robot the gate approach is simpler, more reliable,
  and has zero extra dependencies. Enable ENABLE_BARGE_IN for a middle ground
  where humans can still interrupt.
"""

import threading

import numpy as np
import rclpy
from frida_constants.hri_constants import AEC_AUDIO_TOPIC, RAW_AUDIO_TOPIC, SAYING_TOPIC
from frida_interfaces.msg import AudioData
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

SAMPLE_RATE = 16000  # Hz — matches audio_capturer output


class AcousticEchoCancellation(Node):
    def __init__(self):
        super().__init__("acoustic_echo_cancellation")

        self.declare_parameter("SAYING_TOPIC", SAYING_TOPIC)
        self.declare_parameter("RAW_AUDIO_TOPIC", RAW_AUDIO_TOPIC)
        self.declare_parameter("AEC_AUDIO_TOPIC", AEC_AUDIO_TOPIC)
        # Gain applied while robot is speaking (0.0 = full mute, 0.1 ≈ -20 dB)
        self.declare_parameter("ATTENUATION", 0.0)
        # Ramp length in samples to avoid clicks at gate transitions
        self.declare_parameter("FADE_SAMPLES", 1024)
        # Extra suppression time after speech ends (ms) to catch room reverb tail
        self.declare_parameter("POST_SPEECH_HOLD_MS", 300)
        # Enable barge-in: pass audio through if mic energy >> expected echo level
        self.declare_parameter("ENABLE_BARGE_IN", False)
        # Ratio of mic RMS to running echo RMS that triggers barge-in pass-through
        self.declare_parameter("BARGE_IN_RATIO", 3.0)
        self.declare_parameter("DEBUG", False)

        speaking_topic = self.get_parameter("SAYING_TOPIC").value
        raw_topic = self.get_parameter("RAW_AUDIO_TOPIC").value
        aec_topic = self.get_parameter("AEC_AUDIO_TOPIC").value
        self.attenuation = float(self.get_parameter("ATTENUATION").value)
        self.fade_samples = int(self.get_parameter("FADE_SAMPLES").value)
        post_speech_ms = int(self.get_parameter("POST_SPEECH_HOLD_MS").value)
        self.post_speech_hold_samples = post_speech_ms * SAMPLE_RATE // 1000
        self.enable_barge_in = self.get_parameter("ENABLE_BARGE_IN").value
        self.barge_in_ratio = float(self.get_parameter("BARGE_IN_RATIO").value)
        self.debug = self.get_parameter("DEBUG").value

        # Gate state — protected by _lock
        self._lock = threading.Lock()
        self._speaking = False
        self._post_speech_remaining = 0  # samples still in post-speech hold

        # Gain ramp state (no lock needed — only accessed from audio callback)
        self._current_gain: float = 1.0

        # Barge-in: exponential moving average of mic RMS while robot is speaking
        self._echo_rms_avg: float = 0.0
        _BARGE_IN_EMA_ALPHA = 0.1  # smoothing factor
        self._barge_in_alpha = _BARGE_IN_EMA_ALPHA

        self.publisher_ = self.create_publisher(AudioData, aec_topic, 20)
        self.create_subscription(AudioData, raw_topic, self._audio_callback, 20)
        self.create_subscription(Bool, speaking_topic, self._speaking_callback, 10)

        self.get_logger().info(
            f"AEC ready. {raw_topic} → {aec_topic} | "
            f"attenuation={self.attenuation:.2f}  "
            f"fade={self.fade_samples}smp  "
            f"post_speech={post_speech_ms}ms  "
            f"barge_in={self.enable_barge_in}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _speaking_callback(self, msg: Bool) -> None:
        with self._lock:
            if msg.data and not self._speaking:
                if self.debug:
                    self.get_logger().info(
                        "AEC: robot started speaking — suppressing mic."
                    )
            elif not msg.data and self._speaking:
                # Robot finished speaking: start post-speech hold
                self._post_speech_remaining = self.post_speech_hold_samples
                if self.debug:
                    self.get_logger().info(
                        f"AEC: robot stopped speaking — "
                        f"post-speech hold {self.post_speech_hold_samples} samples."
                    )
            self._speaking = msg.data

    def _audio_callback(self, msg: AudioData) -> None:
        audio = np.frombuffer(msg.data, dtype=np.int16).copy()
        n = len(audio)

        # Determine whether to suppress this chunk
        with self._lock:
            suppress = self._speaking
            if not suppress and self._post_speech_remaining > 0:
                # Still within post-speech suppression window
                self._post_speech_remaining = max(0, self._post_speech_remaining - n)
                suppress = True

        target_gain = self.attenuation if suppress else 1.0

        # Barge-in override: if human voice energy >> expected echo, let audio through
        if suppress and self.enable_barge_in and self.attenuation < 1.0:
            mic_rms = float(np.sqrt(np.mean(audio.astype(np.float32) ** 2)))
            if (
                self._echo_rms_avg > 1.0
                and mic_rms > self._echo_rms_avg * self.barge_in_ratio
            ):
                target_gain = 1.0
                if self.debug:
                    self.get_logger().info(
                        f"AEC barge-in: mic_rms={mic_rms:.0f}  "
                        f"echo_avg={self._echo_rms_avg:.0f}"
                    )
            else:
                # Update echo RMS average using EMA
                self._echo_rms_avg = (
                    1.0 - self._barge_in_alpha
                ) * self._echo_rms_avg + self._barge_in_alpha * mic_rms

        audio = self._apply_fade(audio, target_gain)
        self.publisher_.publish(AudioData(data=audio.tobytes()))

    # ------------------------------------------------------------------
    # Audio helpers
    # ------------------------------------------------------------------

    def _apply_fade(self, audio: np.ndarray, target_gain: float) -> np.ndarray:
        """Apply gain with a linear ramp from _current_gain → target_gain.

        The ramp prevents audible clicks when the gate opens or closes mid-chunk.
        """
        n = len(audio)
        audio_f = audio.astype(np.float32)

        if abs(self._current_gain - target_gain) < 1e-4:
            result = audio_f * self._current_gain
        else:
            ramp_len = min(n, self.fade_samples)
            ramp = np.linspace(
                self._current_gain, target_gain, ramp_len, dtype=np.float32
            )
            result = audio_f.copy()
            result[:ramp_len] *= ramp
            result[ramp_len:] *= target_gain
            self._current_gain = target_gain

        return np.clip(result, -32768, 32767).astype(np.int16)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(AcousticEchoCancellation())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
