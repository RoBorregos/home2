"""ReSpeaker spatial front end: raw mics -> one clean mono stream.

    ch1-4 (raw mics) -> 100 Hz high-pass -> Silero VAD (mic average)
                     -> RTF-MVDR beamformer (noise model learnt in silence)
                     -> [streaming DeepFilterNet3, mild limit] -> gain -> mono

Why not ch0 (the chip's processed beam): in the ReSpeaker noise study ch0 made
Whisper hallucinate / transcribe the interfering talker (WER 200 % with a second
talker) while RTF-MVDR on the raw mics brought it to 7-11 %.

While the robot itself is talking (``saying``) the frames are treated as noise,
so the noise covariance learns the robot's loudspeaker and the beamformer
attenuates it too.

Pure numpy (no ROS): the node ``spatial_audio.py`` and the offline evaluation
(debug/doa/eval_frontend.py) run the same code.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

from speech.spatial.beamformer import MvdrConfig, MvdrRtf
from speech.spatial.denoise import StreamingConfig, StreamingDenoiser
from speech.speaker_id.silero import FRAME_SAMPLES

SR = 16000


@dataclass
class FrontEndConfig:
    hpf_hz: float = 100.0
    vad_on: float = 0.5
    vad_off: float = 0.35
    vad_hangover_sec: float = 0.3
    # raw mics sit well below the chip's AGC'd ch0; Silero likes ch0-like levels
    vad_input_gain_db: float = 12.0
    use_mvdr: bool = True
    output_gain_db: float = 0.0
    mvdr: MvdrConfig = field(default_factory=MvdrConfig)
    dfn: StreamingConfig = field(default_factory=StreamingConfig)


class SpatialFrontEnd:
    def __init__(self, vad, cfg: FrontEndConfig | None = None, enhance_fn=None):
        """vad: callable(512 float32 samples) -> speech probability (SileroVAD).
        enhance_fn: DFN at 48 kHz (see denoise.make_dfn_enhance) or None."""
        self.cfg = c = cfg or FrontEndConfig()
        self.vad = vad
        self.bf = MvdrRtf(4, c.mvdr) if c.use_mvdr else None
        self.den = StreamingDenoiser(enhance_fn, c.dfn) if enhance_fn else None
        self.sos = butter(4, c.hpf_hz, "highpass", fs=SR, output="sos")
        self._zi = np.repeat(sosfilt_zi(self.sos)[:, :, None], 4, axis=2) * 0.0
        self._vad_buf = np.zeros(0, dtype=np.float32)
        self._hang = int(c.vad_hangover_sec * SR / FRAME_SAMPLES)
        self._off_frames = 0
        self.speech = False
        self.last_prob = 0.0
        self.gain = float(10 ** (c.output_gain_db / 20))
        self.vad_gain = float(10 ** (c.vad_input_gain_db / 20))

    def set_enhance_fn(self, fn) -> None:
        """Attach DFN once it has loaded (the stream runs without it meanwhile)."""
        self.den = StreamingDenoiser(fn, self.cfg.dfn) if fn else None

    def _update_vad(self, mono: np.ndarray, saying: bool) -> None:
        self._vad_buf = np.concatenate([self._vad_buf, mono])
        while self._vad_buf.shape[0] >= FRAME_SAMPLES:
            p = self.vad(self._vad_buf[:FRAME_SAMPLES])
            self._vad_buf = self._vad_buf[FRAME_SAMPLES:]
            self.last_prob = p
            if p >= self.cfg.vad_on:
                self.speech = True
                self._off_frames = 0
            elif p < self.cfg.vad_off and self.speech:
                self._off_frames += 1
                if self._off_frames > self._hang:
                    self.speech = False
        if saying:
            self.speech = False

    def process(self, x6: np.ndarray, saying: bool = False) -> np.ndarray:
        """x6: (n, 6) int16 ReSpeaker block. Returns enhanced mono float32."""
        mics = x6[:, 1:5].astype(np.float32) / 32768.0
        mics, self._zi = sosfilt(self.sos, mics, axis=0, zi=self._zi)
        mics = mics.astype(np.float32)
        mono = mics.mean(axis=1)
        # VAD on the beamformer's input; the decision labels these frames
        self._update_vad(np.clip(mono * self.vad_gain, -1, 1), saying)
        if self.bf is not None:
            self.bf.set_speech(self.speech)
            y = self.bf.process(mics)
        else:
            y = mono
        if self.den is not None and y.size:
            y = self.den.process(y)
        return np.clip(y * self.gain, -1.0, 1.0).astype(np.float32)

    @property
    def latency_sec(self) -> float:
        lat = 0.0
        if self.bf is not None:
            lat += (self.cfg.mvdr.n_fft - self.cfg.mvdr.hop) / SR
        if self.den is not None:
            lat += self.den.latency_sec
        return lat
