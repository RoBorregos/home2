#!/usr/bin/env python3
"""
Streaming DSP doorbell detector — no ML, no ROS, round-independent.

Doorbells vary too much (short buzzer, long chime, ding-dong, knock) for any
tonal signature to cover them all without also firing on room sounds. So this
detector confirms on the one thing they share: a **loud sound event** — energy
that rises clearly above the learned ambient floor and lasts a moment.
Specificity comes not from the audio but from the caller's gating (the ROS node
only listens while armed at the door and mutes itself while the robot speaks).

Pipeline: (1) an EMA of quiet frames tracks the ambient floor; (2) frames a
margin above it form an event, confirmed when it lasts >= ``min_event_ms`` with a
peak >= ``confirm_margin_db`` above the floor; (3) tonal features (pitch, clarity,
spectral fingerprint) feed the score and enrollment only, never rejecting.

Feed int16 (or float) mono chunks to ``process()``; it returns confirmed events.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

INT16_MAX = 32768.0


@dataclass
class DoorbellEvent:
    """A confirmed doorbell / loud sound event."""

    time_s: float  # detector time (s) at confirmation
    score: float  # confidence in [0, 1]
    dur_ms: float  # total loud time in the event
    peak_db: float  # loudest frame, dBFS
    margin_db: float  # peak above the ambient floor
    dominant_hz: float  # pitch of the loudest frame (0 if not tonal)
    clarity: float  # mean tonal clarity over the event, [0, 1]
    span_s: float = 0.0  # kept for API compatibility
    spectrum: Optional[np.ndarray] = None  # L2-normalised fingerprint, for enrollment
    template_similarity: float = 0.0  # cosine to the enrolled template, if any


@dataclass
class DoorbellDetectorConfig:
    sample_rate: int = 16000
    frame_ms: float = 24.0  # ~5 periods of the lowest searched pitch (200 Hz)
    hop_ms: float = 12.0  # ~50% overlap

    # Stage 1 — adaptive loudness floor (no fixed min_db).
    active_margin_db: float = 8.0  # a frame joins an event this far above the floor
    floor_alpha: float = 0.05  # EMA weight learning the ambient floor
    silence_floor_db: float = -70.0  # ignore essentially-digital-silence frames

    # Stage 2 — event confirmation by LOUDNESS, not tonal shape. This is what
    # makes detection uniform across every doorbell type and the knock.
    confirm_margin_db: float = 12.0  # the event peak must exceed the floor by this
    event_gap_ms: float = 150.0  # a silence this long ends the event
    min_event_ms: float = 120.0  # total loud time needed to confirm
    max_event_ms: float = 4000.0  # cap so a continuous sound confirms once
    cooldown_s: float = 1.5  # suppress re-confirmations for this long

    # Stage 3 — tonal features, used ONLY for the confidence score and the
    # spectral fingerprint (enrollment). They never reject an event.
    pitch_min_hz: float = 200.0
    pitch_max_hz: float = 1600.0
    clarity_min: float = 0.60  # a frame above this contributes to the tonal score

    # Optional spectral fingerprint / enrollment (round-independent).
    template_bins: int = 64
    template_fmin: float = 150.0
    template_fmax: float = 6000.0
    # A borderline event whose fingerprint matches the enrolled template at or
    # above this cosine similarity is confirmed even below the confidence gate.
    template_sim_confirm: float = 0.85


@dataclass
class _OpenEvent:
    """Mutable accumulator for the loud event currently being built."""

    start_s: float
    last_active_s: float
    floor_db: float  # ambient floor when the event began
    active_frames: int = 0
    peak_db: float = -120.0
    peak_hz: float = 0.0
    clarity_sum: float = 0.0
    clarity_n: int = 0
    spec_sum: Optional[np.ndarray] = None
    spec_n: int = 0

    def add_spectrum(self, mag: np.ndarray) -> None:
        if self.spec_sum is None:
            self.spec_sum = mag.astype(np.float64).copy()
        else:
            self.spec_sum += mag
        self.spec_n += 1


class DoorbellDetector:
    """Streaming, self-calibrating doorbell detector. See module docstring."""

    def __init__(self, config: Optional[DoorbellDetectorConfig] = None, **overrides):
        self.cfg = config or DoorbellDetectorConfig()
        for key, value in overrides.items():
            if not hasattr(self.cfg, key):
                raise AttributeError(f"Unknown DoorbellDetector config field: {key}")
            setattr(self.cfg, key, value)

        cfg = self.cfg
        self.frame_size = max(128, int(cfg.sample_rate * cfg.frame_ms / 1000.0))
        self.hop_size = max(1, int(cfg.sample_rate * cfg.hop_ms / 1000.0))
        self._window = np.hanning(self.frame_size).astype(np.float64)
        self._tau_min = max(1, int(cfg.sample_rate / cfg.pitch_max_hz))
        self._tau_max = min(
            self.frame_size - 1, int(cfg.sample_rate / cfg.pitch_min_hz)
        )
        self._fft_size = 1 << int(np.ceil(np.log2(2 * self.frame_size)))

        # Log-spaced bin edges mapping the rfft magnitude onto a compact, tuning-
        # free fingerprint used for enrollment.
        freqs = np.fft.rfftfreq(self._fft_size, 1.0 / cfg.sample_rate)
        edges = np.geomspace(
            max(1.0, cfg.template_fmin), cfg.template_fmax, cfg.template_bins + 1
        )
        self._bin_idx = np.clip(np.digitize(freqs, edges) - 1, 0, cfg.template_bins - 1)

        self._template: Optional[np.ndarray] = None
        self.reset()

    def reset(self) -> None:
        self._buf = np.zeros(0, dtype=np.float64)
        self._time = 0.0
        self._floor_db: Optional[float] = None
        self._event: Optional[_OpenEvent] = None
        self._cooldown_until = -1e9
        # Diagnostics: every closed event (with why it was accepted/rejected) so a
        # caller can see near-misses and tune. Drained by ``drain_debug()``.
        self._debug: List[dict] = []

    def drain_debug(self) -> List[dict]:
        """Return and clear the event diagnostics gathered so far."""
        out, self._debug = self._debug, []
        return out

    # ── enrollment ───────────────────────────────────────────────────────────

    def enroll_template(self, spectrum: np.ndarray) -> None:
        """Store an event fingerprint as the reference bell for this round."""
        vec = np.asarray(spectrum, dtype=np.float64).ravel()
        norm = np.linalg.norm(vec)
        self._template = vec / norm if norm > 0 else None

    def clear_template(self) -> None:
        self._template = None

    @property
    def has_template(self) -> bool:
        return self._template is not None

    def template_similarity(self, spectrum: Optional[np.ndarray]) -> float:
        if self._template is None or spectrum is None:
            return 0.0
        vec = np.asarray(spectrum, dtype=np.float64).ravel()
        norm = np.linalg.norm(vec)
        if norm == 0:
            return 0.0
        return float(np.clip(np.dot(self._template, vec / norm), 0.0, 1.0))

    # ── per-frame analysis ───────────────────────────────────────────────────

    @staticmethod
    def _to_dbfs(rms: float) -> float:
        return 20.0 * math.log10(max(rms, 1e-9) / INT16_MAX)

    def _spectrum_features(self, frame: np.ndarray):
        """Return (clarity[0,1], pitch_hz, binned_log_magnitude)."""
        x = (frame - frame.mean()) * self._window
        spec = np.fft.rfft(x, self._fft_size)
        mag = np.abs(spec)

        binned = np.zeros(self.cfg.template_bins, dtype=np.float64)
        counts = np.bincount(self._bin_idx, minlength=self.cfg.template_bins)
        np.add.at(binned, self._bin_idx, mag**2)
        binned = np.log1p(binned / np.maximum(counts, 1))

        acf = np.fft.irfft(mag**2, self._fft_size)[: self.frame_size]
        r0 = acf[0]
        if r0 <= 1e-9:
            return 0.0, 0.0, binned
        acf = acf / r0

        seg = acf[self._tau_min : self._tau_max + 1]
        if seg.size == 0:
            return 0.0, 0.0, binned
        k = int(np.argmax(seg))
        tau = self._tau_min + k
        clarity = float(seg[k])

        if 0 < tau < len(acf) - 1:  # parabolic refine for sub-sample lag
            a, b, c = acf[tau - 1], acf[tau], acf[tau + 1]
            denom = a - 2.0 * b + c
            delta = 0.5 * (a - c) / denom if denom != 0 else 0.0
        else:
            delta = 0.0
        pitch = self.cfg.sample_rate / (tau + delta) if (tau + delta) > 0 else 0.0
        return max(0.0, clarity), float(pitch), binned

    def _analyze_frame(self, frame: np.ndarray):
        """Return (active, dbfs, pitch_hz, clarity, spectrum)."""
        cfg = self.cfg
        rms = float(np.sqrt(np.mean(frame**2)) + 1e-9)
        db = self._to_dbfs(rms)

        if self._floor_db is None:
            self._floor_db = db

        active = db > max(self._floor_db + cfg.active_margin_db, cfg.silence_floor_db)
        if not active:
            # Learn the ambient floor only from quiet frames.
            self._floor_db = (
                cfg.floor_alpha * db + (1.0 - cfg.floor_alpha) * self._floor_db
            )
            return False, db, 0.0, 0.0, None

        clarity, pitch, spectrum = self._spectrum_features(frame)
        return True, db, pitch, clarity, spectrum

    # ── streaming entry point ────────────────────────────────────────────────

    def process(self, samples: np.ndarray) -> List[DoorbellEvent]:
        """Feed an int16 (or float) mono chunk; return events confirmed here."""
        samples = np.asarray(samples, dtype=np.float64).reshape(-1)
        self._buf = np.concatenate([self._buf, samples])

        events: List[DoorbellEvent] = []
        while len(self._buf) >= self.frame_size:
            frame = self._buf[: self.frame_size]
            active, db, pitch, clarity, spectrum = self._analyze_frame(frame)

            ev = self._update_event(active, self._time, db, pitch, clarity, spectrum)
            if ev is not None:
                events.append(ev)

            self._buf = self._buf[self.hop_size :]
            self._time += self.hop_size / self.cfg.sample_rate
        return events

    def flush(self) -> List[DoorbellEvent]:
        """Close any open event (e.g. at end of stream) and confirm once more."""
        if self._event is not None:
            ev = self._close_event(self._time)
            if ev is not None:
                return [ev]
        return []

    # ── event building + confirmation ────────────────────────────────────────

    def _update_event(
        self, active, t, db, pitch, clarity, spectrum
    ) -> Optional[DoorbellEvent]:
        cfg = self.cfg
        gap_s = cfg.event_gap_ms / 1000.0

        if active:
            if self._event is None:
                self._event = _OpenEvent(
                    start_s=t, last_active_s=t, floor_db=self._floor_db or db
                )
            e = self._event
            e.last_active_s = t
            e.active_frames += 1
            if db > e.peak_db:
                e.peak_db = db
                e.peak_hz = pitch
            if clarity >= cfg.clarity_min:
                e.clarity_sum += clarity
                e.clarity_n += 1
            if spectrum is not None:
                e.add_spectrum(spectrum)
            # A continuous sound confirms once, then the cooldown suppresses it.
            if (t - e.start_s) * 1000.0 >= cfg.max_event_ms:
                return self._close_event(t)
        elif self._event is not None and (t - self._event.last_active_s) > gap_s:
            return self._close_event(t)

        return None

    def _close_event(self, now: float) -> Optional[DoorbellEvent]:
        e, self._event = self._event, None
        if e is None:
            return None
        cfg = self.cfg
        dur_ms = e.active_frames * cfg.hop_ms
        margin = e.peak_db - e.floor_db
        clarity = e.clarity_sum / max(1, e.clarity_n)

        reason = "ok"
        if dur_ms < cfg.min_event_ms:
            reason = "too_short"
        elif margin < cfg.confirm_margin_db:
            reason = "too_quiet"

        self._debug.append(
            {
                "dur_ms": dur_ms,
                "margin_db": margin,
                "peak_db": e.peak_db,
                "dominant_hz": e.peak_hz,
                "clarity": clarity,
                "reason": reason,
            }
        )
        if reason != "ok" or now < self._cooldown_until:
            return None

        spectrum = None
        if e.spec_sum is not None and e.spec_n > 0:
            vec = e.spec_sum / e.spec_n
            norm = np.linalg.norm(vec)
            spectrum = vec / norm if norm > 0 else vec
        sim = self.template_similarity(spectrum)
        score = self._score(margin, dur_ms, clarity, sim)

        self._cooldown_until = now + cfg.cooldown_s
        return DoorbellEvent(
            time_s=now,
            score=score,
            dur_ms=dur_ms,
            peak_db=e.peak_db,
            margin_db=margin,
            dominant_hz=e.peak_hz,
            clarity=clarity,
            spectrum=spectrum,
            template_similarity=sim,
        )

    def _score(self, margin, dur_ms, clarity, template_sim) -> float:
        """Confidence from loudness, duration and (as a bonus) tonal purity."""
        cfg = self.cfg
        loud = float(np.clip((margin - cfg.confirm_margin_db) / 18.0, 0.0, 1.0))
        dur = float(np.clip(dur_ms / (2.0 * cfg.min_event_ms), 0.0, 1.0))
        tonal = float(np.clip((clarity - 0.5) / 0.4, 0.0, 1.0))
        base = float(np.clip(0.60 + 0.25 * loud + 0.10 * dur + 0.05 * tonal, 0.0, 1.0))
        # A confident template match can only help.
        return float(max(base, 0.5 * base + 0.5 * template_sim))
