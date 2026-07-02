#!/usr/bin/env python3
"""
Streaming DSP "ding-dong" doorbell detector — no ML, no ROS, no tuning.

A chime doorbell is a short sequence of near-pure *tones* that ring and decay.
The classic "ding-dong" is two notes where the second ("dong") sits below the
first ("ding"). Real bells are random: different pitches, spacing and loudness,
in rooms we don't control — so this detector avoids *absolute* thresholds
(fixed frequency band, fixed dB gate, fixed tonality ratio) that would need
per-doorbell tuning. Everything it decides on is adaptive or scale-free:

  1. Adaptive gate: an EMA of the ambient level is tracked continuously. A frame
     is "active" only when it rises a margin (in dB) above that moving floor, so
     it self-calibrates to any room without a fixed ``min_db``.
  2. Pitch + clarity: each active frame is run through autocorrelation. The
     *clarity* (normalised autocorrelation peak, in [0, 1]) tells a periodic tone
     from broadband noise/speech without any energy threshold; the peak lag gives
     the pitch anywhere in a wide range (no narrow band to tune).
  3. Note grouping: consecutive clear frames whose pitch stays stable *relative
     to itself* (a percentage, not a fixed Hz window) become one note. Brief
     dips are tolerated so a decaying ring stays a single note.
  4. Pattern: confirm a "ding-dong" when >= min_tones stable notes fall inside a
     window with the pitch descending (each note lower than the previous). A
     cooldown avoids re-emitting the same ring.

Feed int16 mono chunks to ``process()``; it returns the events confirmed there.
Only numpy is required.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

INT16_MAX = 32768.0


@dataclass
class Tone:
    """A single detected near-pure tone (one note of a chime)."""

    start_s: float  # detector time (s since first sample) the tone began
    end_s: float  # time it ended
    freq_hz: float  # median pitch over the tone
    peak_db: float  # loudest frame level, dBFS
    clarity: float  # mean autocorrelation clarity in [0, 1]

    @property
    def duration_s(self) -> float:
        return max(0.0, self.end_s - self.start_s)


@dataclass
class DingDongEvent:
    """A confirmed ding-dong (or configured chime pattern)."""

    time_s: float  # time of confirmation (end of the last note)
    tones: List[Tone]  # the notes that made up the pattern
    score: float  # confidence in [0, 1]
    span_s: float  # first-onset -> last-onset span

    @property
    def freqs(self) -> List[float]:
        return [t.freq_hz for t in self.tones]


@dataclass
class DingDongDetectorConfig:
    sample_rate: int = 16000
    frame_ms: float = 46.0  # autocorrelation frame (~2 periods of the lowest pitch)
    hop_ms: float = 23.0  # 50% overlap

    # Stage 1 — adaptive loudness gate (no fixed min_db)
    active_margin_db: float = 8.0  # a frame is active this far above the noise floor
    floor_alpha: float = 0.05  # EMA weight learning the ambient floor
    silence_floor_db: float = -70.0  # ignore essentially-digital-silence frames

    # Stage 2 — pitch + clarity (scale-free)
    pitch_min_hz: float = 200.0  # only bounds the lag search; not a narrow band
    pitch_max_hz: float = 4000.0
    clarity_min: float = 0.6  # normalised autocorrelation peak to count as a tone

    # Stage 3 — note grouping (relative, not fixed Hz)
    pitch_tol: float = 0.06  # frames within this fraction of pitch are one note
    note_gap_ms: float = 120.0  # allowed dip inside one note before it ends
    min_note_ms: float = 60.0  # a note must ring at least this long
    max_note_ms: float = 1600.0  # ... and no longer (rejects steady hums)

    # Stage 4 — temporal + pitch pattern
    min_tones: int = 2  # notes needed to confirm (2 == "ding dong")
    min_gap_s: float = 0.03  # min spacing between successive note onsets
    max_gap_s: float = 1.50  # max spacing between successive note onsets
    pattern_window_s: float = 3.5  # all notes must fall inside this window
    require_descending: bool = True  # each note lower than the previous ("ding>dong")
    min_drop_ratio: float = 0.03  # required relative pitch drop when descending
    cooldown_s: float = 1.5  # suppress new confirmations for this long


@dataclass
class _OpenTone:
    """Mutable accumulator for the note currently being built."""

    start_s: float
    last_s: float
    freqs: List[float] = field(default_factory=list)
    peak_db: float = -120.0
    clarity_sum: float = 0.0
    n: int = 0

    def finalize(self) -> Tone:
        return Tone(
            start_s=self.start_s,
            end_s=self.last_s,
            freq_hz=float(np.median(self.freqs)) if self.freqs else 0.0,
            peak_db=self.peak_db,
            clarity=self.clarity_sum / max(1, self.n),
        )


class DingDongDetector:
    """Streaming, self-calibrating ding-dong detector. See module docstring."""

    def __init__(self, config: Optional[DingDongDetectorConfig] = None, **overrides):
        self.cfg = config or DingDongDetectorConfig()
        for key, value in overrides.items():
            if not hasattr(self.cfg, key):
                raise AttributeError(f"Unknown DingDongDetector config field: {key}")
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
        self.reset()

    def reset(self) -> None:
        self._buf = np.zeros(0, dtype=np.float64)
        self._time = 0.0  # time (s) at the start of the pending buffer
        self._floor_db: Optional[float] = None  # adaptive ambient level
        self._open: Optional[_OpenTone] = None
        self._tones: List[Tone] = []
        self._cooldown_until = -1e9

    # ---- per-frame analysis -------------------------------------------------

    @staticmethod
    def _to_dbfs(rms: float) -> float:
        return 20.0 * math.log10(max(rms, 1e-9) / INT16_MAX)

    def _clarity_pitch(self, frame: np.ndarray):
        """Return (clarity in [0,1], pitch_hz) via normalised autocorrelation."""
        x = (frame - frame.mean()) * self._window
        spec = np.fft.rfft(x, self._fft_size)
        acf = np.fft.irfft(np.abs(spec) ** 2, self._fft_size)[: self.frame_size]
        r0 = acf[0]
        if r0 <= 1e-9:
            return 0.0, 0.0
        acf = acf / r0

        seg = acf[self._tau_min : self._tau_max + 1]
        if seg.size == 0:
            return 0.0, 0.0
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
        return max(0.0, clarity), float(pitch)

    def _analyze_frame(self, frame: np.ndarray):
        """Return (is_tone, dbfs, pitch_hz, clarity) with an adaptive gate."""
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
            return False, db, 0.0, 0.0

        clarity, pitch = self._clarity_pitch(frame)
        is_tone = clarity >= cfg.clarity_min and pitch > 0.0
        return is_tone, db, pitch, clarity

    # ---- streaming entry point ----------------------------------------------

    def process(self, samples: np.ndarray) -> List[DingDongEvent]:
        """Feed an int16 (or float) mono chunk; return events confirmed here."""
        samples = np.asarray(samples, dtype=np.float64).reshape(-1)
        self._buf = np.concatenate([self._buf, samples])

        events: List[DingDongEvent] = []
        while len(self._buf) >= self.frame_size:
            frame = self._buf[: self.frame_size]
            frame_time = self._time
            is_tone, db, pitch, clarity = self._analyze_frame(frame)

            ev = self._update_tones(is_tone, frame_time, db, pitch, clarity)
            if ev is not None:
                events.append(ev)

            self._buf = self._buf[self.hop_size :]
            self._time += self.hop_size / self.cfg.sample_rate
        return events

    def flush(self) -> List[DingDongEvent]:
        """Close any open note (e.g. at end of stream) and match once more."""
        if self._open is not None:
            ev = self._close_tone(self._time)
            if ev is not None:
                return [ev]
        return []

    # ---- note grouping + pattern matching -----------------------------------

    def _update_tones(
        self, is_tone: bool, t: float, db: float, pitch: float, clarity: float
    ) -> Optional[DingDongEvent]:
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
            self._open.peak_db = max(self._open.peak_db, db)
            self._open.clarity_sum += clarity
            self._open.n += 1
        elif self._open is not None and (t - self._open.last_s) > note_gap_s:
            return self._close_tone(t)

        return None

    def _close_tone(self, now: float) -> Optional[DingDongEvent]:
        open_tone, self._open = self._open, None
        if open_tone is None:
            return None
        tone = open_tone.finalize()
        dur_ms = tone.duration_s * 1000.0
        if dur_ms < self.cfg.min_note_ms or dur_ms > self.cfg.max_note_ms:
            return None
        self._tones.append(tone)
        return self._match_pattern(now)

    def _match_pattern(self, now: float) -> Optional[DingDongEvent]:
        cfg = self.cfg
        # Keep only notes whose onset is inside the pattern window.
        self._tones = [
            t for t in self._tones if now - t.start_s <= cfg.pattern_window_s
        ]
        if now < self._cooldown_until or len(self._tones) < cfg.min_tones:
            return None

        seq = self._tones[-cfg.min_tones :]
        onsets = [t.start_s for t in seq]
        span = onsets[-1] - onsets[0]
        if span > cfg.pattern_window_s:
            return None

        for prev, cur in zip(seq, seq[1:]):
            gap = cur.start_s - prev.start_s
            if not (cfg.min_gap_s <= gap <= cfg.max_gap_s):
                return None
            if cfg.require_descending and cur.freq_hz > prev.freq_hz * (
                1.0 - cfg.min_drop_ratio
            ):
                return None

        score = self._score(seq)
        self._cooldown_until = now + cfg.cooldown_s
        self._tones = []  # consume the matched notes
        return DingDongEvent(time_s=now, tones=list(seq), score=score, span_s=span)

    def _score(self, seq: List[Tone]) -> float:
        """Confidence from mean clarity and (if required) pitch-drop cleanliness."""
        clarity = float(np.mean([t.clarity for t in seq]))
        if self.cfg.require_descending and len(seq) > 1:
            drops = [
                (a.freq_hz - b.freq_hz) / max(1.0, a.freq_hz)
                for a, b in zip(seq, seq[1:])
            ]
            pitch = float(np.clip(np.mean(drops) * 4.0, 0.0, 1.0))
            return float(np.clip(0.6 * clarity + 0.4 * pitch, 0.0, 1.0))
        return float(np.clip(clarity, 0.0, 1.0))
