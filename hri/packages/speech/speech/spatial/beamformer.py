"""Streaming MVDR beamformer steered by the relative transfer function (RTF).

Front end recommended by the ReSpeaker noise study (RTF-MVDR, "covariance
whitening"): +4-5 dB SNR and about -26 WER points over a raw mic at >= 90 deg
talker/interferer separation, where muting or averaging mics gains ~0.4 dB.

Per STFT bin k:
    Rn  noise covariance, averaged over frames the VAD marks as silence
    Ry  covariance of speech frames (talker + noise), shorter memory
    h   RTF = Rn v, v = principal generalised eigenvector of (Ry, Rn),
        normalised to the reference mic -> no geometry and no DOA needed,
        reverberation included
    w   = Rn^-1 h / (h^H Rn^-1 h)          (distortionless toward the talker)

The output is the talker as heard at the reference mic, with noise from other
directions attenuated. Until both covariances are learnt the output is the mic
average, so the stream is usable from the first chunk.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class MvdrConfig:
    sample_rate: int = 16000
    n_fft: int = 512
    hop: int = 128
    ref_mic: int = 0
    noise_tau_sec: float = 3.0
    speech_tau_sec: float = 1.5
    loading: float = 1e-3  # diagonal loading relative to trace(Rn)/M
    update_every: int = 4  # frames between weight updates
    min_noise_frames: int = 30
    min_speech_frames: int = 15


class MvdrRtf:
    def __init__(self, n_mics: int = 4, cfg: MvdrConfig | None = None):
        self.cfg = c = cfg or MvdrConfig()
        self.m = n_mics
        self.n_bins = c.n_fft // 2 + 1
        # periodic Hann analysis + synthesis at 75 % overlap: sum(w^2) = 1.5
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
        """mics: (n, M) float32 in [-1, 1]. Returns the beamformed mono samples
        available so far (latency n_fft - hop samples)."""
        c = self.cfg
        self._in = np.concatenate([self._in, mics.astype(np.float32)], axis=0)
        out = []
        while self._in.shape[0] >= c.n_fft:
            frame = self._in[: c.n_fft] * self.win[:, None]
            self._in = self._in[c.hop :]
            X = np.fft.rfft(frame, axis=0).astype(np.complex64)  # (K, M)
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
            u = vecs[:, :, -1]  # principal eigenvector of the whitened Ry
            h = np.einsum("kij,kj->ki", L, u)  # de-whiten: h = L u
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
