"""ReSpeaker 4-mic array: MVDR beamformer."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

from speech.silero_vad_utils import VadGate

SR = 16000


@dataclass
class MvdrConfig:
    sample_rate: int = 16000
    n_fft: int = 512
    hop: int = 128
    ref_mic: int = 0
    noise_tau_sec: float = 3.0
    speech_tau_sec: float = 1.5
    loading: float = 1e-3
    update_every: int = 4
    min_noise_frames: int = 30
    min_speech_frames: int = 15


class MvdrRtf:
    def __init__(self, n_mics: int = 4, cfg: MvdrConfig | None = None):
        self.cfg = c = cfg or MvdrConfig()
        self.m = n_mics
        self.n_bins = c.n_fft // 2 + 1
        win = np.hanning(c.n_fft + 1)[:-1]
        self.win = win.astype(np.float32)
        self.norm = float(np.sum(win**2) / c.hop)
        self._in = np.zeros((0, n_mics), dtype=np.float32)
        self._ola = np.zeros(c.n_fft, dtype=np.float32)
        eye = np.eye(n_mics, dtype=np.complex64)
        self.Rn = np.tile(eye * 1e-6, (self.n_bins, 1, 1))
        self.Ry = self.Rn.copy()
        self.n_noise = 0
        self.n_speech = 0
        self.w = np.full((self.n_bins, n_mics), 1.0 / n_mics, dtype=np.complex64)
        self._an = 1.0 - np.exp(-c.hop / (c.noise_tau_sec * c.sample_rate))
        self._as = 1.0 - np.exp(-c.hop / (c.speech_tau_sec * c.sample_rate))
        self._since_update = 0
        self.speech = False
        self.ready = False

    def set_speech(self, active: bool) -> None:
        self.speech = bool(active)

    def process(self, mics: np.ndarray) -> np.ndarray:
        c = self.cfg
        self._in = np.concatenate([self._in, mics.astype(np.float32)], axis=0)
        out = []
        while self._in.shape[0] >= c.n_fft:
            frame = self._in[: c.n_fft] * self.win[:, None]
            self._in = self._in[c.hop :]
            X = np.fft.rfft(frame, axis=0).astype(np.complex64)
            cov = X[:, :, None] * X[:, None, :].conj()
            if self.speech:
                self.Ry += self._as * (cov - self.Ry)
                self.n_speech += 1
            else:
                self.Rn += self._an * (cov - self.Rn)
                self.n_noise += 1
            self._since_update += 1
            if (
                self._since_update >= c.update_every
                and self.n_noise >= c.min_noise_frames
                and self.n_speech >= c.min_speech_frames
            ):
                self._update_weights()
                self._since_update = 0
            Y = np.einsum("km,km->k", self.w.conj(), X)
            y = np.fft.irfft(Y, c.n_fft).astype(np.float32) * self.win / self.norm
            self._ola += y
            out.append(self._ola[: c.hop].copy())
            self._ola = np.concatenate(
                [self._ola[c.hop :], np.zeros(c.hop, dtype=np.float32)]
            )
        return np.concatenate(out) if out else np.zeros(0, dtype=np.float32)

    def _update_weights(self) -> None:
        c = self.cfg
        M = self.m
        tr = np.real(np.trace(self.Rn, axis1=1, axis2=2)) / M
        Rn = self.Rn + (c.loading * tr + 1e-10)[:, None, None] * np.eye(M)
        try:
            L = np.linalg.cholesky(Rn)
            Linv = np.linalg.inv(L)
            C = Linv @ self.Ry @ Linv.conj().transpose(0, 2, 1)
            C = 0.5 * (C + C.conj().transpose(0, 2, 1))
            _, vecs = np.linalg.eigh(C)
            u = vecs[:, :, -1]
            h = np.einsum("kij,kj->ki", L, u)
            ref = h[:, c.ref_mic : c.ref_mic + 1]
            h = h / np.where(np.abs(ref) < 1e-9, 1e-9, ref)
            Rinv_h = np.linalg.solve(Rn, h[:, :, None])[:, :, 0]
            den = np.einsum("km,km->k", h.conj(), Rinv_h)
            w = Rinv_h / np.where(np.abs(den) < 1e-12, 1e-12, den)[:, None]
            if np.all(np.isfinite(w)):
                self.w = w.astype(np.complex64)
                self.ready = True
        except np.linalg.LinAlgError:
            pass


@dataclass
class FrontEndConfig:
    hpf_hz: float = 100.0
    vad_on: float = 0.5
    vad_off: float = 0.35
    vad_hangover_sec: float = 0.3
    vad_input_gain_db: float = 12.0
    output_gain_db: float = 0.0
    mvdr: MvdrConfig = field(default_factory=MvdrConfig)


class MvdrFrontEnd:
    def __init__(self, vad, cfg: FrontEndConfig | None = None):
        self.cfg = c = cfg or FrontEndConfig()
        self.gate = VadGate(vad, SR, c.vad_on, c.vad_off, c.vad_hangover_sec)
        self.bf = MvdrRtf(4, c.mvdr)
        self.sos = butter(4, c.hpf_hz, "highpass", fs=SR, output="sos")
        self._zi = np.repeat(sosfilt_zi(self.sos)[:, :, None], 4, axis=2) * 0.0
        self.gain = float(10 ** (c.output_gain_db / 20))
        self.vad_gain = float(10 ** (c.vad_input_gain_db / 20))

    @property
    def speech(self) -> bool:
        return self.gate.speech

    def process(self, x6: np.ndarray, saying: bool = False) -> np.ndarray:
        mics = x6[:, 1:5].astype(np.float32) / 32768.0
        mics, self._zi = sosfilt(self.sos, mics, axis=0, zi=self._zi)
        mics = mics.astype(np.float32)
        mono = np.clip(mics.mean(axis=1) * self.vad_gain, -1, 1)
        self.bf.set_speech(self.gate.update(mono, saying))
        y = self.bf.process(mics)
        return np.clip(y * self.gain, -1.0, 1.0).astype(np.float32)
