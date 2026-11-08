#!/usr/bin/env python3
"""
Streaming door-event (doorbell / knock) detector backed by a pretrained
AudioSet tagger — no ROS, so it can be tested offline.

Why a pretrained tagger: the hand-crafted detectors (pitch tracker, tone shape,
knock rhythm) and the small Edge Impulse model all failed to generalise across
doorbell types (buzzer, ding-dong, chime, melody, electronic). EfficientAT
``mn10_as`` was trained on ~2M AudioSet clips whose labels include Doorbell,
Ding-dong, Bell, Chime, Buzzer, Knock and Tap, so it has already heard
thousands of different bells and knocks.

Pipeline: keep the last ``window_s`` of 16 kHz audio; every ``hop_s`` resample
it to 32 kHz and tag it. Per keyword (doorbell / knock) the probabilities of its
AudioSet classes are summed: a real bell spreads its mass over neighbouring
classes (Doorbell, Ding-dong, Tubular bells, Alarm clock, Glockenspiel...), while
speech and room noise put ~0 on all of them. A window "hits" when the larger
keyword score reaches ``threshold``. A detection is confirmed after
``min_consecutive`` hits in a row, or at once if a window reaches
``strong_threshold``. Feed int16 (or float) mono chunks to ``process()``.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
import torch
from scipy.signal import resample_poly

from speech.audioset_labels import AUDIOSET_LABELS
from speech.efficientat import SAMPLE_RATE as MODEL_SR
from speech.efficientat import AudioTagger

INT16_MAX = 32768.0

# Old and melodic bells are often tagged as their neighbours rather than
# "Doorbell" (e.g. a mechanical "tring" as Alarm clock, a chime as Tubular bells).
DEFAULT_DOORBELL_CLASSES = [
    "Doorbell",
    "Ding-dong",
    "Ding",
    "Bell",
    "Chime",
    "Buzzer",
    "Ringtone",
    "Tubular bells",
    "Alarm clock",
    "Bicycle bell",
    "Telephone bell ringing",
    "Jingle bell",
    "Wind chime",
    "Glockenspiel",
]
DEFAULT_KNOCK_CLASSES = ["Knock", "Tap"]


@dataclass
class DoorEvent:
    """A confirmed door event."""

    time_s: float  # detector time (s) at confirmation
    keyword: str  # "doorbell" or "knock"
    label: str  # most likely AudioSet class within the keyword
    score: float  # summed keyword probability, clipped to [0, 1]
    speech_score: float  # AudioSet "Speech" probability of the same window


@dataclass
class DoorEventTaggerConfig:
    sample_rate: int = 16000
    # Audio tagged per inference. mn10_as needs >= ~3.2 s of input: shorter
    # windows saturate to p=1.0 on arbitrary classes (verified against the
    # upstream EfficientAT code). A short ring is still detected one hop after
    # it starts; it just occupies part of the window.
    window_s: float = 5.0
    hop_s: float = 0.5  # time between inferences
    threshold: float = 0.15  # per-window hit on the summed keyword score
    strong_threshold: float = 0.5  # a single window this sure confirms at once
    min_consecutive: int = 2  # hits in a row needed below strong_threshold
    cooldown_s: float = 2.0  # suppress re-confirmations for this long
    min_db: float = -55.0  # skip inference on windows quieter than this dBFS
    doorbell_classes: List[str] = field(
        default_factory=lambda: list(DEFAULT_DOORBELL_CLASSES)
    )
    knock_classes: List[str] = field(
        default_factory=lambda: list(DEFAULT_KNOCK_CLASSES)
    )
    doorbell_keyword: str = "doorbell"
    knock_keyword: str = "knock"
    weights_path: str = ""
    device: str = ""  # "" = cuda if available, else cpu


def _class_indices(names: List[str]) -> Dict[str, int]:
    index = {name: i for i, name in enumerate(AUDIOSET_LABELS)}
    unknown = [n for n in names if n not in index]
    if unknown:
        raise ValueError(f"Unknown AudioSet class name(s): {unknown}")
    return {n: index[n] for n in names}


class DoorEventTagger:
    """Streaming doorbell/knock detector. See module docstring."""

    def __init__(
        self,
        config: Optional[DoorEventTaggerConfig] = None,
        tagger: Optional[AudioTagger] = None,
    ):
        self.cfg = cfg = config or DoorEventTaggerConfig()
        if tagger is None:
            if not os.path.isfile(cfg.weights_path):
                raise FileNotFoundError(
                    f"EfficientAT weights not found at '{cfg.weights_path}'. "
                    "Run docker/hri/scripts/download-model.sh (efficientat)."
                )
            tagger = AudioTagger(cfg.weights_path, device=cfg.device or None)
        self.tagger = tagger

        self._doorbell_idx = _class_indices(cfg.doorbell_classes)
        self._knock_idx = _class_indices(cfg.knock_classes)
        self._speech_idx = _class_indices(["Speech"])["Speech"]

        g = math.gcd(MODEL_SR, cfg.sample_rate)
        self._up, self._down = MODEL_SR // g, cfg.sample_rate // g
        self.window_size = int(cfg.sample_rate * cfg.window_s)
        self.hop_size = max(1, int(cfg.sample_rate * cfg.hop_s))
        self.reset()

    def reset(self) -> None:
        """Drop buffered audio and pending hits (e.g. on re-arm or after speaking)."""
        self._buf = np.zeros(0, dtype=np.float32)
        self._since_infer = 0
        self._time = 0.0
        self._hits = 0
        self._cooldown_until = -1e9
        # (keyword, label, door score, speech score) of the last tagged window.
        self.last_scores: Optional[tuple] = None

    # ── scoring ──────────────────────────────────────────────────────────────

    def score_window(self, window: np.ndarray) -> np.ndarray:
        """Tag one float window (input sample rate, [-1, 1]); return 527 probabilities."""
        x = resample_poly(window, self._up, self._down).astype(np.float32)
        probs, _ = self.tagger(torch.from_numpy(x).unsqueeze(0))
        return probs[0].numpy()

    def best_door_class(self, probs: np.ndarray):
        """Return (keyword, top class label, summed score) of the likelier keyword."""
        best = ("", "", 0.0)
        for keyword, idx in (
            (self.cfg.doorbell_keyword, self._doorbell_idx),
            (self.cfg.knock_keyword, self._knock_idx),
        ):
            score = min(1.0, float(sum(probs[i] for i in idx.values())))
            if score > best[2]:
                label = max(idx, key=lambda name: probs[idx[name]])
                best = (keyword, label, score)
        return best

    # ── streaming entry point ────────────────────────────────────────────────

    def process(self, samples: np.ndarray) -> List[DoorEvent]:
        """Feed an int16 (or float in [-1, 1]) mono chunk; return confirmed events."""
        samples = np.asarray(samples).reshape(-1)
        if samples.dtype == np.int16:
            samples = samples.astype(np.float32) / INT16_MAX
        else:
            samples = samples.astype(np.float32)

        self._buf = np.concatenate([self._buf, samples])[-self.window_size :]
        self._since_infer += len(samples)
        self._time += len(samples) / self.cfg.sample_rate

        events: List[DoorEvent] = []
        # Infer at most once per call so a burst of queued chunks can't stall it.
        if self._since_infer >= self.hop_size:
            self._since_infer = 0
            # Right after (re)arming the buffer is short: left-pad with silence
            # so a ring is caught without waiting for a full window.
            window = self._buf
            if len(window) < self.window_size:
                window = np.pad(window, (self.window_size - len(window), 0))
            event = self._step(window)
            if event is not None:
                events.append(event)
        return events

    def _step(self, window: np.ndarray) -> Optional[DoorEvent]:
        cfg = self.cfg
        rms = float(np.sqrt(np.mean(window**2)) + 1e-9)
        if 20.0 * math.log10(rms) < cfg.min_db:
            self._hits = 0
            return None

        probs = self.score_window(window)
        keyword, label, score = self.best_door_class(probs)
        self.last_scores = (keyword, label, score, float(probs[self._speech_idx]))

        if score < cfg.threshold:
            self._hits = 0
            return None
        self._hits += 1
        if self._time < self._cooldown_until:
            return None
        if score < cfg.strong_threshold and self._hits < cfg.min_consecutive:
            return None

        self._hits = 0
        self._cooldown_until = self._time + cfg.cooldown_s
        return DoorEvent(
            time_s=self._time,
            keyword=keyword,
            label=label,
            score=score,
            speech_score=float(probs[self._speech_idx]),
        )
