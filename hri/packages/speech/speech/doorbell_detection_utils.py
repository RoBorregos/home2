#!/usr/bin/env python3
"""
Streaming DSP doorbell ("chime") detector — no ML, no ROS, round-independent.

Unlike ``ding_dong_detection_utils`` (which required a *descending two-note*
pattern and so both missed non-descending / single-tone bells and fired on the
falling intonation of party speech), this detector confirms a bell from the
acoustic *signature of a ringing tone*, which speech does not produce:

  * a near-pure tone (high autocorrelation clarity),
  * whose pitch stays stable for a sustained time (>= ``sustain_min_ms``) —
    speech pitch drifts and is broken by consonants within ~100-200 ms, and
  * whose energy then decays (ring-down) — a struck/electronic chime rings and
    fades, a held vowel plateaus.

Any number of tones, in any pitch order, confirms it, so single-tone buzzers,
ascending and multi-note chimes all register. Everything is adaptive or
scale-free (adaptive loudness floor, relative pitch tolerance), so no
per-doorbell tuning is needed.

Optional enrollment (see ``enroll_template`` / ``template_similarity``): the
per-event log-magnitude spectrum is a compact fingerprint. In this task the
bell is the *same within a round* (it rings for both guests), so the first
confirmed ring can be enrolled and used to raise confidence on the second —
handling a doorbell whose sound changes between rounds without any manual setup.

Feed int16 (or float) mono chunks to ``process()``; it returns confirmed events.
Only numpy is required.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

INT16_MAX = 32768.0


@dataclass
class DoorbellTone:
    """A single sustained, decaying near-pure tone (one chime note)."""

    start_s: float
    end_s: float
    freq_hz: float
    peak_db: float
    clarity: float
    decay_ratio: float  # early-RMS / late-RMS; > 1 means the tone rings down

    @property
    def duration_s(self) -> float:
        return max(0.0, self.end_s - self.start_s)


@dataclass
class DoorbellEvent:
    """A confirmed doorbell ring."""

    time_s: float
    tones: List[DoorbellTone]
    score: float
    span_s: float
    spectrum: Optional[np.ndarray] = None  # L2-normalised fingerprint, for enrollment
    template_similarity: float = 0.0  # cosine to the enrolled template, if any

    @property
    def freqs(self) -> List[float]:
        return [t.freq_hz for t in self.tones]


@dataclass
class DoorbellDetectorConfig:
    sample_rate: int = 16000
    frame_ms: float = 24.0  # ~5 periods of the lowest searched pitch (200 Hz)
    hop_ms: float = 12.0  # ~50% overlap

    # Stage 1 — adaptive loudness gate (no fixed min_db). A bell is meant to be
    # heard, so require a clear margin over the learned ambient floor.
    active_margin_db: float = 10.0
    floor_alpha: float = 0.05
    silence_floor_db: float = -70.0

    # Stage 2 — pitch + clarity (scale-free). clarity_min is high: a chime is a
    # near-pure tone, which rejects the lower-clarity, formant-rich vowels of
    # speech.
    pitch_min_hz: float = 200.0
    pitch_max_hz: float = 4000.0
    clarity_min: float = 0.80

    # Stage 3 — grouping frames into one ringing note.
    pitch_tol: float = 0.06  # frames within this fraction of pitch are one note
    note_gap_ms: float = 80.0  # tolerated dip inside one note
    sustain_min_ms: float = 140.0  # a tone must ring this long (rejects speech)
    max_note_ms: float = 2500.0  # ... and no longer (rejects steady hums)
    min_decay_ratio: float = 1.25  # early/late RMS: the tone must fade, not plateau

    # Stage 4 — event confirmation. min_tones=1 accepts single-tone buzzers; the
    # sustain + decay + purity gates above are what reject speech, not the count.
    min_tones: int = 1
    min_gap_s: float = 0.03
    max_gap_s: float = 2.0
    pattern_window_s: float = 3.5
    cooldown_s: float = 1.5

    # Stage 5 — optional spectral fingerprint / enrollment (round-independent).
    template_bins: int = 64
    template_fmin: float = 150.0
    template_fmax: float = 6000.0
    # A borderline event whose fingerprint matches the enrolled template at or
    # above this cosine similarity is confirmed even if its raw score is low.
    template_sim_confirm: float = 0.85


@dataclass
class _OpenTone:
    """Mutable accumulator for the note currently being built."""

    start_s: float
    last_s: float
    freqs: List[float] = field(default_factory=list)
    rms: List[float] = field(default_factory=list)
    peak_db: float = -120.0
    clarity_sum: float = 0.0
    n: int = 0
    spec_sum: Optional[np.ndarray] = None

    def add_spectrum(self, mag: np.ndarray) -> None:
        if self.spec_sum is None:
            self.spec_sum = mag.astype(np.float64).copy()
        else:
            self.spec_sum += mag

    def decay_ratio(self) -> float:
        """early-RMS / late-RMS over the note; > 1 means it rings down."""
        n = len(self.rms)
        if n < 3:
            return 1.0
        third = max(1, n // 3)
        early = float(np.mean(self.rms[:third]))
        late = float(np.mean(self.rms[-third:]))
        return early / max(late, 1e-9)

    def finalize(self, decay: float) -> DoorbellTone:
        return DoorbellTone(
            start_s=self.start_s,
            end_s=self.last_s,
            freq_hz=float(np.median(self.freqs)) if self.freqs else 0.0,
            peak_db=self.peak_db,
            clarity=self.clarity_sum / max(1, self.n),
            decay_ratio=decay,
        )


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

        # Log-spaced bin edges mapping the rfft magnitude spectrum onto a compact,
        # tuning-free fingerprint used for enrollment.
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
        self._open: Optional[_OpenTone] = None
        self._tones: List[DoorbellTone] = []
        self._tone_specs: List[np.ndarray] = []  # fingerprint per accepted tone
        self._cooldown_until = -1e9

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

        # Compact log-magnitude fingerprint (per-bin mean power, log-compressed).
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
        """Return (is_tone, dbfs, rms, pitch_hz, clarity, spectrum)."""
        cfg = self.cfg
        rms = float(np.sqrt(np.mean(frame**2)) + 1e-9)
        db = self._to_dbfs(rms)

        if self._floor_db is None:
            self._floor_db = db

        active = db > max(self._floor_db + cfg.active_margin_db, cfg.silence_floor_db)
        if not active:
            self._floor_db = (
                cfg.floor_alpha * db + (1.0 - cfg.floor_alpha) * self._floor_db
            )
            return False, db, rms, 0.0, 0.0, None

        clarity, pitch, spectrum = self._spectrum_features(frame)
        is_tone = clarity >= cfg.clarity_min and pitch > 0.0
        return is_tone, db, rms, pitch, clarity, spectrum

    # ── streaming entry point ────────────────────────────────────────────────

    def process(self, samples: np.ndarray) -> List[DoorbellEvent]:
        """Feed an int16 (or float) mono chunk; return events confirmed here."""
        samples = np.asarray(samples, dtype=np.float64).reshape(-1)
        self._buf = np.concatenate([self._buf, samples])

        events: List[DoorbellEvent] = []
        while len(self._buf) >= self.frame_size:
            frame = self._buf[: self.frame_size]
            t = self._time
            is_tone, db, rms, pitch, clarity, spectrum = self._analyze_frame(frame)

            ev = self._update_tones(is_tone, t, db, rms, pitch, clarity, spectrum)
            if ev is not None:
                events.append(ev)

            self._buf = self._buf[self.hop_size :]
            self._time += self.hop_size / self.cfg.sample_rate
        return events

    def flush(self) -> List[DoorbellEvent]:
        """Close any open note (e.g. at end of stream) and match once more."""
        if self._open is not None:
            ev = self._close_tone(self._time)
            if ev is not None:
                return [ev]
        return []

    # ── note grouping + confirmation ─────────────────────────────────────────

    def _update_tones(
        self, is_tone, t, db, rms, pitch, clarity, spectrum
    ) -> Optional[DoorbellEvent]:
        cfg = self.cfg
        note_gap_s = cfg.note_gap_ms / 1000.0

        if is_tone:
            ref = np.median(self._open.freqs) if self._open else 0.0
            same = (
                self._open is not None
                and (t - self._open.last_s) <= note_gap_s
                and abs(pitch - ref) <= cfg.pitch_tol * max(ref, 1.0)
            )
            if not same and self._open is not None:
                ev = self._close_tone(t)
                if ev is not None:
                    return ev
            if self._open is None:
                self._open = _OpenTone(start_s=t, last_s=t)
            self._open.last_s = t
            self._open.freqs.append(pitch)
            self._open.rms.append(rms)
            self._open.peak_db = max(self._open.peak_db, db)
            self._open.clarity_sum += clarity
            self._open.n += 1
            if spectrum is not None:
                self._open.add_spectrum(spectrum)
        elif self._open is not None and (t - self._open.last_s) > note_gap_s:
            return self._close_tone(t)

        return None

    def _close_tone(self, now: float) -> Optional[DoorbellEvent]:
        open_tone, self._open = self._open, None
        if open_tone is None:
            return None
        cfg = self.cfg
        dur_ms = (open_tone.last_s - open_tone.start_s) * 1000.0
        decay = open_tone.decay_ratio()
        # A chime note must be sustained, bounded and ring down. These three are
        # the discriminators against speech; a note that fails is discarded.
        if (
            dur_ms < cfg.sustain_min_ms
            or dur_ms > cfg.max_note_ms
            or decay < cfg.min_decay_ratio
        ):
            return None

        self._tones.append(open_tone.finalize(decay))
        n = max(1, open_tone.n)
        self._tone_specs.append(
            open_tone.spec_sum / n
            if open_tone.spec_sum is not None
            else np.zeros(cfg.template_bins)
        )
        return self._match_pattern(now)

    def _match_pattern(self, now: float) -> Optional[DoorbellEvent]:
        cfg = self.cfg
        # Keep only tones whose onset is inside the pattern window.
        keep = [
            i
            for i, t in enumerate(self._tones)
            if now - t.start_s <= cfg.pattern_window_s
        ]
        self._tones = [self._tones[i] for i in keep]
        self._tone_specs = [self._tone_specs[i] for i in keep]

        if now < self._cooldown_until or len(self._tones) < cfg.min_tones:
            return None

        seq = self._tones[-cfg.min_tones :]
        specs = self._tone_specs[-cfg.min_tones :]
        onsets = [t.start_s for t in seq]
        span = onsets[-1] - onsets[0]
        if span > cfg.pattern_window_s:
            return None
        for prev, cur in zip(onsets, onsets[1:]):
            gap = cur - prev
            if not (cfg.min_gap_s <= gap <= cfg.max_gap_s):
                return None

        spectrum = self._event_fingerprint(specs)
        sim = self.template_similarity(spectrum)
        score = self._score(seq, sim)

        self._cooldown_until = now + cfg.cooldown_s
        self._tones = []
        self._tone_specs = []
        return DoorbellEvent(
            time_s=now,
            tones=list(seq),
            score=score,
            span_s=span,
            spectrum=spectrum,
            template_similarity=sim,
        )

    @staticmethod
    def _event_fingerprint(specs: List[np.ndarray]) -> Optional[np.ndarray]:
        if not specs:
            return None
        vec = np.mean(np.stack(specs, axis=0), axis=0)
        norm = np.linalg.norm(vec)
        return vec / norm if norm > 0 else vec

    def _score(self, seq: List[DoorbellTone], template_sim: float) -> float:
        """Confidence from tone purity, sustain, ring-down and template match."""
        cfg = self.cfg
        clarity = float(np.mean([t.clarity for t in seq]))
        purity = float(
            np.clip((clarity - cfg.clarity_min) / (1.0 - cfg.clarity_min), 0.0, 1.0)
        )
        sustain = float(
            np.clip(
                np.mean([t.duration_s for t in seq])
                / (2.0 * cfg.sustain_min_ms / 1000.0),
                0.0,
                1.0,
            )
        )
        decay = float(
            np.clip(
                (np.mean([t.decay_ratio for t in seq]) - 1.0)
                / max(1e-6, cfg.min_decay_ratio - 1.0)
                * 0.5,
                0.0,
                1.0,
            )
        )
        base = 0.45 * purity + 0.35 * sustain + 0.20 * decay
        # A confident template match lifts the score toward its similarity; it can
        # only help (an unenrolled or non-matching bell keeps the DSP-only base).
        return float(np.clip(max(base, 0.5 * base + 0.5 * template_sim), 0.0, 1.0))
