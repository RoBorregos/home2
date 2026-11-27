"""ReSpeaker 4-mic array: MVDR beamformer and SRP-PHAT direction."""

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


SPEED_OF_SOUND = 343.0
RESPEAKER_V2_RADIUS = 0.032
RESPEAKER_V2_MIC_ANGLES_DEG = (0.0, 90.0, 180.0, 270.0)


def wrap180(deg):
    out = (np.asarray(deg, dtype=np.float64) + 180.0) % 360.0 - 180.0
    out = np.where(out == -180.0, 180.0, out)
    return float(out) if np.ndim(out) == 0 else out


def circ_diff(a, b):
    return wrap180(np.asarray(a) - np.asarray(b))


@dataclass
class ArrayGeometry:
    radius: float = RESPEAKER_V2_RADIUS
    mic_angles_deg: tuple = RESPEAKER_V2_MIC_ANGLES_DEG
    offset_deg: float = 0.0
    mirror: bool = False

    @property
    def positions(self) -> np.ndarray:
        a = np.radians(np.asarray(self.mic_angles_deg, dtype=np.float64))
        return np.stack([self.radius * np.cos(a), self.radius * np.sin(a)], axis=1)

    def to_robot(self, array_deg):
        a = np.asarray(array_deg, dtype=np.float64)
        return wrap180((-a if self.mirror else a) + self.offset_deg)


@dataclass
class DoaConfig:
    sample_rate: int = 16000
    n_fft: int = 512
    hop: int = 256
    f_min: float = 250.0
    f_max: float = 3800.0
    grid_step_deg: float = 2.0
    noise_tau_sec: float = 4.0
    noise_sub: float = 1.0
    min_frame_snr_db: float = 3.0
    second_peak_ratio: float = 0.6
    min_separation_deg: float = 40.0
    max_sources: int = 2
    geometry: ArrayGeometry = field(default_factory=ArrayGeometry)


@dataclass
class DoaEstimate:
    azimuth_deg: float
    array_deg: float
    confidence: float
    power: float
    frames: int
    sources: list = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "azimuth_deg": round(self.azimuth_deg, 1),
            "array_deg": round(self.array_deg, 1),
            "confidence": round(self.confidence, 3),
            "frames": self.frames,
            "sources": [
                {"azimuth_deg": round(a, 1), "rel": round(r, 3)}
                for a, r in self.sources
            ],
        }


class SrpPhat:
    def __init__(self, cfg: DoaConfig | None = None, history_sec: float = 16.0):
        self.cfg = cfg or DoaConfig()
        c = self.cfg
        self.window = np.sqrt(np.hanning(c.n_fft + 1)[:-1]).astype(np.float32)
        freqs = np.fft.rfftfreq(c.n_fft, 1.0 / c.sample_rate)
        self.bins = np.where((freqs >= c.f_min) & (freqs <= c.f_max))[0]
        self.freqs = freqs[self.bins]
        self.grid = np.arange(0.0, 360.0, c.grid_step_deg)
        pos = c.geometry.positions
        self.n_mics = pos.shape[0]
        self.pairs = [
            (i, j) for i in range(self.n_mics) for j in range(i + 1, self.n_mics)
        ]
        th = np.radians(self.grid)
        u = np.stack([np.cos(th), np.sin(th)], axis=1)
        tau = -(u @ pos.T) / SPEED_OF_SOUND
        dtau = np.stack([tau[:, i] - tau[:, j] for i, j in self.pairs], axis=1)
        w = 2 * np.pi * self.freqs
        self.steer = np.exp(1j * w[None, :, None] * dtau[:, None, :]).astype(
            np.complex64
        )
        self.max_frames = int(history_sec * c.sample_rate / c.hop)
        self._buf = np.zeros((0, self.n_mics), dtype=np.float32)
        self._frames = []
        self.noise_cov = None
        self.noise_floor_db = None
        self._alpha = 1.0 - np.exp(-c.hop / (c.noise_tau_sec * c.sample_rate))
        self._speech_hint = None

    def set_speech(self, active: bool | None) -> None:
        self._speech_hint = active

    def push(self, mics: np.ndarray) -> None:
        x = np.asarray(mics, dtype=np.float32)
        if x.dtype != np.float32 or np.abs(x).max(initial=0) > 2.0:
            x = x / 32768.0
        self._buf = np.concatenate([self._buf, x], axis=0)
        c = self.cfg
        while self._buf.shape[0] >= c.n_fft:
            seg = self._buf[: c.n_fft] * self.window[:, None]
            self._buf = self._buf[c.hop :]
            spec = np.fft.rfft(seg, axis=0)[self.bins].astype(np.complex64)
            power = float(np.mean(np.abs(spec) ** 2)) + 1e-12
            db = 10 * np.log10(power)
            if self.noise_floor_db is None:
                self.noise_floor_db = db
            if self._speech_hint is None:
                speech = db > self.noise_floor_db + 6.0
            else:
                speech = bool(self._speech_hint)
            if not speech:
                cov = np.einsum("km,kn->kmn", spec, spec.conj())
                if self.noise_cov is None:
                    self.noise_cov = cov
                else:
                    self.noise_cov += self._alpha * (cov - self.noise_cov)
                self.noise_floor_db += self._alpha * (db - self.noise_floor_db)
            else:
                self.noise_floor_db = min(self.noise_floor_db, db + 20.0)
            self._frames.append((spec, db, speech))
            if len(self._frames) > self.max_frames:
                self._frames.pop(0)

    def _cov(self, frames) -> tuple[np.ndarray | None, int]:
        c = self.cfg
        floor = self.noise_floor_db if self.noise_floor_db is not None else -120.0
        sel = [f for f in frames if f[1] >= floor + c.min_frame_snr_db]
        if not sel:
            return None, 0
        X = np.stack([f[0] for f in sel], axis=0)
        R = np.einsum("fkm,fkn->kmn", X, X.conj()) / X.shape[0]
        if self.noise_cov is not None and c.noise_sub > 0:
            R = R - c.noise_sub * self.noise_cov
        return R, len(sel)

    def srp_map(self, R: np.ndarray) -> np.ndarray:
        G = np.stack([R[:, i, j] for i, j in self.pairs], axis=1)
        G = G / (np.abs(G) + 1e-12)
        return np.real(np.einsum("gkp,kp->g", self.steer, G)) / G.size

    def localize_frames(self, frames) -> DoaEstimate | None:
        R, n = self._cov(frames)
        if R is None:
            return None
        return self._estimate(self.srp_map(R), n)

    def localize(self, n_samples: int) -> DoaEstimate | None:
        n = max(1, int(np.ceil(n_samples / self.cfg.hop)))
        return self.localize_frames(self._frames[-n:])

    def _estimate(self, srp: np.ndarray, n_frames: int) -> DoaEstimate:
        c = self.cfg
        peaks = _circular_peaks(srp)
        order = sorted(peaks, key=lambda i: -srp[i])
        best = order[0]
        top = srp[best]
        spread = float(np.std(srp)) + 1e-9
        conf = float(np.clip((top - float(np.median(srp))) / (4.0 * spread), 0, 1))
        geo = c.geometry
        sources = [(float(geo.to_robot(self.grid[best])), 1.0)]
        for i in order[1:]:
            if len(sources) >= c.max_sources:
                break
            rel = float(srp[i] / top) if top > 0 else 0.0
            if rel < c.second_peak_ratio:
                break
            if all(
                abs(circ_diff(self.grid[i], self.grid[j])) >= c.min_separation_deg
                for j in [best]
            ):
                sources.append((float(geo.to_robot(self.grid[i])), rel))
        return DoaEstimate(
            azimuth_deg=float(geo.to_robot(self.grid[best])),
            array_deg=float(self.grid[best]),
            confidence=conf,
            power=float(top),
            frames=n_frames,
            sources=sources,
        )


def _circular_peaks(x: np.ndarray) -> list[int]:
    left = np.roll(x, 1)
    right = np.roll(x, -1)
    idx = np.where((x >= left) & (x > right))[0]
    return list(idx) if idx.size else [int(np.argmax(x))]


_SECTORS = (
    (22.5, "front"),
    (67.5, "front-left"),
    (112.5, "left"),
    (157.5, "back-left"),
    (180.1, "back"),
)


def direction_label(robot_deg: float) -> str:
    a = float(wrap180(robot_deg))
    for lim, name in _SECTORS:
        if abs(a) <= lim:
            if a < 0 and "left" in name:
                return name.replace("left", "right")
            return name
    return "back"


def merge_doas(doas: list[dict], min_conf: float) -> dict | None:
    doas = [d for d in doas if d]
    if not doas:
        return None
    good = [d for d in doas if d["confidence"] >= min_conf] or doas
    w = np.array([max(d["confidence"], 1e-3) for d in good])
    az = np.radians([d["azimuth_deg"] for d in good])
    mean = float(np.degrees(np.angle(np.sum(w * np.exp(1j * az)))))
    conf = float(np.mean([d["confidence"] for d in good]))
    return {
        "azimuth_deg": round(mean, 1),
        "confidence": round(conf, 3),
        "frames": len(good),
        "sources": max(good, key=lambda d: d["confidence"]).get("sources", []),
        "direction": direction_label(mean),
        "reliable": conf >= min_conf,
    }
