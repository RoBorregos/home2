#!/usr/bin/env python3
"""
Mathematical (DSP) knock detector — no machine learning, no ROS dependency.

A knock is an impulsive, broadband-but-low-frequency transient: a sharp energy
onset that decays quickly, usually repeated 2-4 times within a second or two.
This module detects it purely with signal processing so it can run anywhere
(standalone on a laptop mic, or inside a ROS node) without an Edge Impulse
server.

Pipeline (mirrors the stages described in the plan):
  1. Energy / dB gate ("filtro 1"): per-frame RMS -> dBFS; frames quieter than
     ``min_db`` are ignored before any further analysis.
  2. Transient / onset detection: optional band-pass (knock energy ~ 60-1000 Hz)
     then an energy-envelope onset detector that fires when a frame's energy
     jumps sharply above an adaptive baseline. A refractory period prevents a
     single knock from registering as several onsets.
  3. Temporal pattern: a knock is *confirmed* when at least ``min_onsets`` onsets
     occur inside ``pattern_window_s`` (spaced at least ``min_onset_gap_s``
     apart). A cooldown avoids re-emitting the same knock burst.

The detector is streaming: feed it arbitrary-length int16 mono chunks via
``process()`` and it returns any knock events confirmed during that chunk.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

try:
    from scipy.signal import butter, lfilter, lfilter_zi

    _HAS_SCIPY = True
except Exception:  # pragma: no cover - scipy is normally present
    _HAS_SCIPY = False

INT16_MAX = 32768.0


@dataclass
class KnockEvent:
    """A confirmed knock burst."""

    time_s: float  # detector time (seconds since first sample) of confirmation
    num_onsets: int  # how many onsets made up the burst
    score: float  # confidence in [0, 1]
    peak_db: float  # loudest onset level in dBFS


@dataclass
class KnockDetectorConfig:
    sample_rate: int = 16000
    frame_ms: float = 16.0  # envelope frame length (~256 samples @ 16 kHz)
    # Stage 1 — energy / dB gate ("filtro 1")
    min_db: float = -40.0  # frames below this dBFS are ignored
    # Stage 2 — onset detection
    bandpass_low_hz: float = 60.0
    bandpass_high_hz: float = 1000.0
    use_bandpass: bool = True
    onset_ratio: float = 3.0  # frame energy must exceed baseline * ratio
    baseline_alpha: float = 0.10  # EMA weight for the adaptive noise baseline
    refractory_s: float = 0.08  # min spacing between two onsets
    # Stage 3 — temporal pattern
    min_onsets: int = 2  # onsets needed to confirm a knock burst
    pattern_window_s: float = 1.5  # onsets must fall inside this window
    min_onset_gap_s: float = 0.10  # knocks are spaced at least this far apart
    cooldown_s: float = 1.0  # suppress new confirmations for this long


class KnockDetector:
    """Streaming DSP knock detector. See module docstring for the pipeline."""

    def __init__(self, config: Optional[KnockDetectorConfig] = None, **overrides):
        self.cfg = config or KnockDetectorConfig()
        for key, value in overrides.items():
            if not hasattr(self.cfg, key):
                raise AttributeError(f"Unknown KnockDetector config field: {key}")
            setattr(self.cfg, key, value)

        cfg = self.cfg
        self.frame_size = max(1, int(cfg.sample_rate * cfg.frame_ms / 1000.0))

        # Band-pass state (Butterworth, applied with persistent filter memory)
        self._b = self._a = self._zi = None
        if cfg.use_bandpass and _HAS_SCIPY:
            nyq = cfg.sample_rate / 2.0
            low = max(1e-4, cfg.bandpass_low_hz / nyq)
            high = min(0.999, cfg.bandpass_high_hz / nyq)
            if low < high:
                self._b, self._a = butter(2, [low, high], btype="band")
                self._zi = lfilter_zi(self._b, self._a)

        self.reset()

    def reset(self) -> None:
        self._pending = np.zeros(0, dtype=np.float32)  # leftover samples < 1 frame
        self._baseline = None  # adaptive energy baseline (RMS)
        self._samples_seen = 0
        self._last_onset_time: Optional[float] = None
        self._onset_times: List[float] = []
        self._onset_levels: List[float] = []  # dBFS per onset in current window
        self._last_confirm_time: float = -1e9
        if self._zi is not None:
            self._filter_state = self._zi.copy()
        else:
            self._filter_state = None

    # ── helpers ────────────────────────────────────────────────────────────

    @staticmethod
    def _rms(frame: np.ndarray) -> float:
        return float(np.sqrt(np.mean(frame.astype(np.float64) ** 2)) + 1e-9)

    @staticmethod
    def _to_dbfs(rms: float) -> float:
        return 20.0 * math.log10(max(rms, 1e-9) / INT16_MAX)

    def _bandpass(self, samples_f: np.ndarray) -> np.ndarray:
        if self._b is None or self._filter_state is None:
            return samples_f
        out, self._filter_state = lfilter(
            self._b, self._a, samples_f, zi=self._filter_state
        )
        return out

    # ── main API ───────────────────────────────────────────────────────────

    def process(self, samples: np.ndarray) -> List[KnockEvent]:
        """Feed an int16 mono chunk; return knock bursts confirmed in this chunk."""
        cfg = self.cfg
        samples_f = np.asarray(samples, dtype=np.float32).ravel()
        if samples_f.size == 0:
            return []

        # Band-pass emphasises knock energy and rejects hiss / sibilance.
        samples_f = self._bandpass(samples_f)

        buf = (
            np.concatenate([self._pending, samples_f])
            if self._pending.size
            else samples_f
        )
        n_frames = buf.size // self.frame_size
        events: List[KnockEvent] = []

        for i in range(n_frames):
            frame = buf[i * self.frame_size : (i + 1) * self.frame_size]
            self._samples_seen += self.frame_size
            frame_time = self._samples_seen / cfg.sample_rate

            rms = self._rms(frame)
            db = self._to_dbfs(rms)

            if self._baseline is None:
                self._baseline = rms

            # Stage 1 — energy / dB gate ("filtro 1")
            if db < cfg.min_db:
                # Quiet frame: let the noise baseline track it, no onset possible.
                self._baseline = (
                    1 - cfg.baseline_alpha
                ) * self._baseline + cfg.baseline_alpha * rms
                self._expire_onsets(frame_time)
                continue

            # Stage 2 — onset: sharp rise above adaptive baseline
            is_onset = rms > self._baseline * cfg.onset_ratio
            if is_onset and (
                self._last_onset_time is None
                or (frame_time - self._last_onset_time) >= cfg.refractory_s
            ):
                event = self._register_onset(frame_time, db)
                if event is not None:
                    events.append(event)
            else:
                # Track background level only while *not* onsetting.
                self._baseline = (
                    1 - cfg.baseline_alpha
                ) * self._baseline + cfg.baseline_alpha * rms

            self._expire_onsets(frame_time)

        self._pending = buf[n_frames * self.frame_size :].copy()
        return events

    # ── stage 3 — temporal pattern ──────────────────────────────────────────

    def _register_onset(self, frame_time: float, db: float) -> Optional[KnockEvent]:
        cfg = self.cfg
        # Enforce a minimum gap between the knocks that make up a burst.
        if (
            self._onset_times
            and (frame_time - self._onset_times[-1]) < cfg.min_onset_gap_s
        ):
            return None

        self._last_onset_time = frame_time
        self._onset_times.append(frame_time)
        self._onset_levels.append(db)

        if len(self._onset_times) < cfg.min_onsets:
            return None
        if (frame_time - self._last_confirm_time) < cfg.cooldown_s:
            return None

        # Confirmed knock burst.
        num = len(self._onset_times)
        peak_db = max(self._onset_levels)
        # Score: blend onset count and how far above the gate the loudest hit is.
        loudness = float(np.clip((peak_db - cfg.min_db) / 40.0, 0.0, 1.0))
        count_score = float(np.clip(num / (cfg.min_onsets + 1), 0.0, 1.0))
        score = float(np.clip(0.5 * loudness + 0.5 * count_score, 0.0, 1.0))

        self._last_confirm_time = frame_time
        self._onset_times.clear()
        self._onset_levels.clear()
        return KnockEvent(
            time_s=frame_time, num_onsets=num, score=score, peak_db=peak_db
        )

    def _expire_onsets(self, now: float) -> None:
        """Drop onsets older than the pattern window so bursts don't merge."""
        window = self.cfg.pattern_window_s
        while self._onset_times and (now - self._onset_times[0]) > window:
            self._onset_times.pop(0)
            self._onset_levels.pop(0)
