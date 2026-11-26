"""Spatial audio utilities for the ReSpeaker 4-mic array (issue #1459), used by
``scripts/speaker_verification.py``. Pure numpy (no ROS):

- SRP-PHAT direction of arrival on the raw mics (ch1-4), robot frame,
- speaker tracker: remembers which enrolled person spoke from which direction,
- RTF-MVDR beamformer and ``MvdrFrontEnd`` (raw mics -> one clean mono stream).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi


# --------------------------------------------------------------------------
# Direction of arrival (SRP-PHAT)
# Direction of arrival (DOA) on the ReSpeaker USB 4-mic array (raw ch1-4).
#
# SRP-PHAT on the *speech* spatial covariance: the covariance of the analysed
# window minus the noise covariance learned while nobody speaks (the "silence
# model"). Subtracting the noise model keeps steady room noise (fans, the robot's
# own motors, a TV that was already on) from pulling the map toward itself.
#
# Angles are returned in the robot frame by ``ArrayGeometry.to_robot``:
# 0 deg = front of the robot, +90 = its left, -90 = its right (ROS, CCW),
# in (-180, 180]. The array frame (``ARRAY_*``) is the one the mic positions
# below are written in; ``offset_deg`` / ``mirror`` map it onto the robot and
# are calibrated on the robot (see config/sound_localization.yaml).
# --------------------------------------------------------------------------

SPEED_OF_SOUND = 343.0
# ReSpeaker USB Mic Array v2.0 (XVF-3000, 6-channel firmware): ch1..ch4 are the
# raw mics on a 32 mm radius circle, 90 deg apart, in order around the circle.
RESPEAKER_V2_RADIUS = 0.032
RESPEAKER_V2_MIC_ANGLES_DEG = (0.0, 90.0, 180.0, 270.0)


def wrap180(deg):
    """Wrap degrees to (-180, 180]."""
    out = (np.asarray(deg, dtype=np.float64) + 180.0) % 360.0 - 180.0
    out = np.where(out == -180.0, 180.0, out)
    return float(out) if np.ndim(out) == 0 else out


def circ_diff(a, b):
    """Smallest signed difference a - b in degrees."""
    return wrap180(np.asarray(a) - np.asarray(b))


@dataclass
class ArrayGeometry:
    radius: float = RESPEAKER_V2_RADIUS
    mic_angles_deg: tuple = RESPEAKER_V2_MIC_ANGLES_DEG
    offset_deg: float = 0.0  # robot angle of the array's 0 deg axis
    mirror: bool = False  # array angles increase clockwise seen from above

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
    # Above ~3.8 kHz the 45 mm adjacent pairs alias; keep the band below it.
    f_max: float = 3800.0
    grid_step_deg: float = 2.0
    # Noise model: exponential average of the covariance of non-speech frames.
    noise_tau_sec: float = 4.0
    # Fraction of the noise covariance subtracted (1.0 = full silence model).
    noise_sub: float = 1.0
    # Frames whose energy is < this many dB above the noise floor are ignored
    # when localising a window.
    min_frame_snr_db: float = 3.0
    # A second source is reported when its SRP peak is >= this fraction of the
    # first and >= min_separation_deg away from it.
    second_peak_ratio: float = 0.6
    min_separation_deg: float = 40.0
    max_sources: int = 2
    geometry: ArrayGeometry = field(default_factory=ArrayGeometry)


@dataclass
class DoaEstimate:
    azimuth_deg: float  # robot frame
    array_deg: float  # array frame (before offset/mirror), for calibration
    confidence: float  # 0..1 peak sharpness of the SRP map
    power: float  # relative peak value
    frames: int  # frames that contributed
    sources: list = field(default_factory=list)  # [(robot_deg, rel_power)]

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
    """Streaming SRP-PHAT with a running noise covariance.

    ``push(mics)`` takes int16/float audio shaped (n, 4) and keeps the STFT
    frames; ``localize(n_samples)`` localises the last ``n_samples``. Frames are
    labelled speech/non-speech by the caller through ``set_speech`` (the
    speaker-id VAD) or, if never called, by an energy gate over the noise floor.
    """

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
        # tau_m(theta): arrival time of a far-field wave from theta at mic m,
        # relative to the array centre (earlier for mics facing the source).
        th = np.radians(self.grid)
        u = np.stack([np.cos(th), np.sin(th)], axis=1)  # (G, 2)
        tau = -(u @ pos.T) / SPEED_OF_SOUND  # (G, M)
        dtau = np.stack([tau[:, i] - tau[:, j] for i, j in self.pairs], axis=1)
        # steering for R_ij = E[X_i X_j*] ~ exp(-j w (tau_i - tau_j))
        w = 2 * np.pi * self.freqs
        self.steer = np.exp(1j * w[None, :, None] * dtau[:, None, :]).astype(
            np.complex64
        )  # (G, K, P)

        self.max_frames = int(history_sec * c.sample_rate / c.hop)
        self._buf = np.zeros((0, self.n_mics), dtype=np.float32)
        self._frames = []  # list of (X (K, M) complex64, db, speech flag)
        self.noise_cov = None  # (K, M, M)
        self.noise_floor_db = None
        self._alpha = 1.0 - np.exp(-c.hop / (c.noise_tau_sec * c.sample_rate))
        self._speech_hint = None
        self.total_frames = 0

    # ----------------------------------------------------------- streaming
    def set_speech(self, active: bool | None) -> None:
        """External VAD hint for the frames pushed from now on (None = auto)."""
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
            spec = np.fft.rfft(seg, axis=0)[self.bins].astype(np.complex64)  # (K, M)
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
                # let the floor creep down if it started on speech
                self.noise_floor_db = min(self.noise_floor_db, db + 20.0)
            self._frames.append((spec, db, speech))
            self.total_frames += 1
            if len(self._frames) > self.max_frames:
                self._frames.pop(0)

    # ------------------------------------------------------------ analysis
    def _cov(self, frames) -> tuple[np.ndarray | None, int]:
        c = self.cfg
        floor = self.noise_floor_db if self.noise_floor_db is not None else -120.0
        sel = [f for f in frames if f[1] >= floor + c.min_frame_snr_db]
        if not sel:
            return None, 0
        X = np.stack([f[0] for f in sel], axis=0)  # (F, K, M)
        R = np.einsum("fkm,fkn->kmn", X, X.conj()) / X.shape[0]
        if self.noise_cov is not None and c.noise_sub > 0:
            R = R - c.noise_sub * self.noise_cov
        return R, len(sel)

    def srp_map(self, R: np.ndarray) -> np.ndarray:
        G = np.stack([R[:, i, j] for i, j in self.pairs], axis=1)  # (K, P)
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
        # confidence: how far the peak stands over the map's spread (0..1)
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


def srp_offline(
    mics: np.ndarray, cfg: DoaConfig | None = None, win_sec: float = 1.0
) -> list[tuple[float, DoaEstimate | None]]:
    """Localise a (n, 4) recording window by window (for tests and analysis)."""
    cfg = cfg or DoaConfig()
    srp = SrpPhat(cfg, history_sec=win_sec + 1)
    step = int(win_sec * cfg.sample_rate)
    out = []
    for s in range(0, mics.shape[0] - step + 1, step):
        srp.push(mics[s : s + step])
        out.append((s / cfg.sample_rate, srp.localize(step)))
    return out


_SECTORS = (
    (22.5, "front"),
    (67.5, "front-left"),
    (112.5, "left"),
    (157.5, "back-left"),
    (180.1, "back"),
)


def direction_label(robot_deg: float) -> str:
    """Coarse human-readable direction for a robot-frame azimuth."""
    a = float(wrap180(robot_deg))
    for lim, name in _SECTORS:
        if abs(a) <= lim:
            if a < 0 and "left" in name:
                return name.replace("left", "right")
            return name
    return "back"


def merge_doas(doas: list[dict], min_conf: float) -> dict | None:
    """Confidence-weighted circular mean of several window DOAs (``_doa_for``
    dicts). Used to give one direction to the target windows of a segment that
    mixes several voices. Reliable windows are preferred when there are any."""
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
        "sources": [],
        "direction": direction_label(mean),
        "reliable": conf >= min_conf,
    }


# --------------------------------------------------------------------------
# Speaker tracker
# Associate voice identities with directions over time.
#
# The speaker embedding is unsure on short, turned-away or far phrases (UNKNOWN
# band between REJECT_TH and ACCEPT_TH). People rarely teleport, so a voice that
# comes from where a known person was confidently identified a moment ago is
# most likely still that person. The tracker keeps one track per identified
# person (name, azimuth, last confident time) and resolves each decision:
#
#     voice confident (TARGET-level score)      -> that name, via "voice";
#                                                  the track moves to this DOA
#     voice unsure (between reject and accept),
#     DOA near a fresh track                    -> track's name, via "direction"
#     voice rejected (<= reject threshold)      -> "unknown" even at a track's DOA
#     otherwise                                 -> "unknown"
#
# Only confident voice decisions create or move tracks, so a wrong direction
# match cannot snowball into more wrong matches.
# --------------------------------------------------------------------------


@dataclass
class Track:
    name: str
    azimuth_deg: float
    last_t: float
    hits: int = 1


@dataclass
class TrackerConfig:
    max_age_sec: float = 8.0  # a track is usable this long after its last hit
    match_deg: float = 25.0  # DOA gate for attributing an unsure voice
    smooth: float = 0.5  # azimuth update weight for a new confident hit


class SpeakerTracker:
    def __init__(self, cfg: TrackerConfig | None = None):
        self.cfg = cfg or TrackerConfig()
        self.tracks: dict[str, Track] = {}

    def _fresh(self, now: float) -> list[Track]:
        return [
            t for t in self.tracks.values() if now - t.last_t <= self.cfg.max_age_sec
        ]

    def update(
        self,
        now: float,
        name: str | None,
        confident: bool,
        azimuth_deg: float | None,
        doa_reliable: bool = True,
        rejected: bool = False,
    ) -> tuple[str, str]:
        """Returns (speaker, via) with via in {"voice", "direction", "none"}.

        ``rejected``: the voice scored at/below the reject threshold for every
        profile, i.e. it is positively *not* a known person -> never attributed
        by direction (someone else may be standing in the same place)."""
        if confident and name:
            if azimuth_deg is not None and doa_reliable:
                tr = self.tracks.get(name)
                if tr is None or now - tr.last_t > self.cfg.max_age_sec:
                    self.tracks[name] = Track(name, azimuth_deg, now)
                else:
                    step = self.cfg.smooth * float(
                        circ_diff(azimuth_deg, tr.azimuth_deg)
                    )
                    tr.azimuth_deg = float(circ_diff(tr.azimuth_deg + step, 0.0))
                    tr.last_t = now
                    tr.hits += 1
            elif name in self.tracks:
                self.tracks[name].last_t = now
            return name, "voice"
        if rejected or azimuth_deg is None or not doa_reliable:
            return "unknown", "none"
        best, best_d = None, None
        for tr in self._fresh(now):
            d = abs(float(circ_diff(azimuth_deg, tr.azimuth_deg)))
            if d <= self.cfg.match_deg and (best_d is None or d < best_d):
                best, best_d = tr, d
        if best is None:
            return "unknown", "none"
        return best.name, "direction"

    def snapshot(self, now: float) -> list[dict]:
        return [
            {
                "name": t.name,
                "azimuth_deg": round(t.azimuth_deg, 1),
                "age_sec": round(now - t.last_t, 1),
                "hits": t.hits,
            }
            for t in self._fresh(now)
        ]


# --------------------------------------------------------------------------
# RTF-MVDR beamformer
# Streaming MVDR beamformer steered by the relative transfer function (RTF).
#
# Front end recommended by the ReSpeaker noise study (RTF-MVDR, "covariance
# whitening"): +4-5 dB SNR and about -26 WER points over a raw mic at >= 90 deg
# talker/interferer separation, where muting or averaging mics gains ~0.4 dB.
#
# Per STFT bin k:
#     Rn  noise covariance, averaged over frames the VAD marks as silence
#     Ry  covariance of speech frames (talker + noise), shorter memory
#     h   RTF = Rn v, v = principal generalised eigenvector of (Ry, Rn),
#         normalised to the reference mic -> no geometry and no DOA needed,
#         reverberation included
#     w   = Rn^-1 h / (h^H Rn^-1 h)          (distortionless toward the talker)
#
# The output is the talker as heard at the reference mic, with noise from other
# directions attenuated. Until both covariances are learnt the output is the mic
# average, so the stream is usable from the first chunk.
# --------------------------------------------------------------------------


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


# --------------------------------------------------------------------------
# MVDR front end
# ch1-4 (raw mics) -> 100 Hz high-pass -> Silero VAD on the mic average
#                  -> RTF-MVDR (noise model learnt in silence) -> gain -> mono
# While the robot itself talks (``saying``) frames are treated as noise, so the
# beamformer also learns to attenuate the robot's loudspeaker.
# On the lab recordings it raised the target's worst speaker-id score from
# 0.46 (chip ch0) to 0.59 and d' from 7.7 to 9.5.
# --------------------------------------------------------------------------

SR = 16000
VAD_FRAME = 512  # Silero VAD frame (32 ms at 16 kHz)


@dataclass
class FrontEndConfig:
    hpf_hz: float = 100.0
    vad_on: float = 0.5
    vad_off: float = 0.35
    vad_hangover_sec: float = 0.3
    # raw mics sit well below the chip's AGC'd ch0; Silero likes ch0-like levels
    vad_input_gain_db: float = 12.0
    output_gain_db: float = 0.0
    mvdr: MvdrConfig = field(default_factory=MvdrConfig)


class MvdrFrontEnd:
    def __init__(self, vad, cfg: FrontEndConfig | None = None):
        """vad: callable(512 float32 samples) -> speech probability (SileroVAD)."""
        self.cfg = c = cfg or FrontEndConfig()
        self.vad = vad
        self.bf = MvdrRtf(4, c.mvdr)
        self.sos = butter(4, c.hpf_hz, "highpass", fs=SR, output="sos")
        self._zi = np.repeat(sosfilt_zi(self.sos)[:, :, None], 4, axis=2) * 0.0
        self._vad_buf = np.zeros(0, dtype=np.float32)
        self._hang = int(c.vad_hangover_sec * SR / VAD_FRAME)
        self._off_frames = 0
        self.speech = False
        self.gain = float(10 ** (c.output_gain_db / 20))
        self.vad_gain = float(10 ** (c.vad_input_gain_db / 20))

    def _update_vad(self, mono: np.ndarray, saying: bool) -> None:
        self._vad_buf = np.concatenate([self._vad_buf, mono])
        while self._vad_buf.shape[0] >= VAD_FRAME:
            p = self.vad(self._vad_buf[:VAD_FRAME])
            self._vad_buf = self._vad_buf[VAD_FRAME:]
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
        """x6: (n, 6) int16 ReSpeaker block. Returns the beamformed mono float32."""
        mics = x6[:, 1:5].astype(np.float32) / 32768.0
        mics, self._zi = sosfilt(self.sos, mics, axis=0, zi=self._zi)
        mics = mics.astype(np.float32)
        # VAD on the beamformer's input; the decision labels these frames
        self._update_vad(np.clip(mics.mean(axis=1) * self.vad_gain, -1, 1), saying)
        self.bf.set_speech(self.speech)
        y = self.bf.process(mics)
        return np.clip(y * self.gain, -1.0, 1.0).astype(np.float32)
