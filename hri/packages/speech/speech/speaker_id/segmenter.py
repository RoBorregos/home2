"""Speech segmentation on top of Silero probabilities.

Per 32 ms frame:

* hysteresis on the Silero probability (``vad_on`` to open, ``vad_off`` to keep),
* adaptive energy gate: to *open* a segment the frame must be ``energy_margin_db``
  above a rolling noise floor (EMA of non-speech frames),
* ``min_speech_sec`` of consecutive speech to open, ``min_silence_sec`` to close,
* ``pad_sec`` of audio kept before the onset and after the offset,
* ``mute()`` (robot TTS) closes the current segment and suppresses new ones.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class SegmenterConfig:
    sample_rate: int = 16000
    frame_samples: int = 512
    vad_on: float = 0.5
    vad_off: float = 0.4
    energy_margin_db: float = 3.0
    noise_floor_tau_sec: float = 3.0
    min_speech_sec: float = 0.25
    min_silence_sec: float = 0.5
    pad_sec: float = 0.2
    max_segment_sec: float = 15.0


@dataclass
class FrameInfo:
    """Per-frame diagnostics returned by :meth:`Segmenter.push`."""

    prob: float
    db: float
    noise_floor_db: float
    speech: bool  # segmenter state after this frame
    started: bool = False
    ended: bool = False
    muted: bool = False


@dataclass
class Segment:
    seg_id: int
    t_start: float  # stream time (s) of the first speech frame (without pad)
    audio: list[np.ndarray] = field(default_factory=list)
    probs: list[float] = field(default_factory=list)
    dbs: list[float] = field(default_factory=list)
    t_end: float = 0.0
    cut_by_tts: bool = False

    def wave(self) -> np.ndarray:
        if not self.audio:
            return np.zeros(0, dtype=np.float32)
        return np.concatenate(self.audio)

    def duration(self, sample_rate: int) -> float:
        return sum(a.shape[0] for a in self.audio) / sample_rate


def frame_db(frame: np.ndarray) -> float:
    rms = float(np.sqrt(np.mean(np.square(frame, dtype=np.float64)) + 1e-12))
    return 20.0 * np.log10(rms + 1e-9)


class Segmenter:
    def __init__(self, cfg: SegmenterConfig) -> None:
        self.cfg = cfg
        frame_sec = cfg.frame_samples / cfg.sample_rate
        self.frame_sec = frame_sec
        self._min_speech = max(1, round(cfg.min_speech_sec / frame_sec))
        self._min_silence = max(1, round(cfg.min_silence_sec / frame_sec))
        self._pad = max(0, round(cfg.pad_sec / frame_sec))
        self._max_frames = max(1, round(cfg.max_segment_sec / frame_sec))
        self._alpha = min(1.0, frame_sec / max(cfg.noise_floor_tau_sec, 1e-3))
        self._pre: deque[tuple[np.ndarray, float, float]] = deque(
            maxlen=self._pad + self._min_speech
        )
        self.noise_floor_db = -60.0
        self._floor_init = False
        self.t = 0.0  # stream time in seconds
        self.in_speech = False
        self._onset_run = 0
        self._silence_run = 0
        self._next_id = 1
        self.current: Segment | None = None
        self._muted = False

    # ------------------------------------------------------------------ API
    def mute(self, muted: bool) -> Segment | None:
        """Mute (robot speaking). Closes and returns an open segment, if any."""
        self._muted = muted
        if muted and self.current is not None:
            seg = self._close(cut_by_tts=True)
            return seg
        return None

    def push(
        self, frame: np.ndarray, prob: float
    ) -> tuple[FrameInfo, Segment | None, Segment | None]:
        """Feed one frame. Returns (info, started_segment, finished_segment)."""
        self.t += self.frame_sec
        db = frame_db(frame)
        started: Segment | None = None
        finished: Segment | None = None

        if self._muted:
            self._pre.clear()
            self._onset_run = 0
            info = FrameInfo(prob, db, self.noise_floor_db, False, muted=True)
            return info, None, None

        if not self.in_speech:
            self._update_floor(db, prob)
            loud = db >= self.noise_floor_db + self.cfg.energy_margin_db
            self._pre.append((frame, prob, db))
            if prob >= self.cfg.vad_on and loud:
                self._onset_run += 1
            else:
                self._onset_run = 0
            if self._onset_run >= self._min_speech:
                started = self._open()
        else:
            seg = self.current
            assert seg is not None
            seg.audio.append(frame)
            seg.probs.append(prob)
            seg.dbs.append(db)
            if prob >= self.cfg.vad_off:
                self._silence_run = 0
            else:
                self._silence_run += 1
            if self._silence_run >= self._min_silence:
                # keep only `pad` frames of the trailing silence
                drop = max(0, self._silence_run - self._pad)
                if drop:
                    del seg.audio[-drop:]
                    del seg.probs[-drop:]
                    del seg.dbs[-drop:]
                finished = self._close()
            elif len(seg.audio) >= self._max_frames:
                finished = self._close()

        info = FrameInfo(
            prob,
            db,
            self.noise_floor_db,
            self.in_speech,
            started=started is not None,
            ended=finished is not None,
        )
        return info, started, finished

    # -------------------------------------------------------------- helpers
    def _update_floor(self, db: float, prob: float) -> None:
        if prob >= self.cfg.vad_off:
            return
        if not self._floor_init:
            self.noise_floor_db = db
            self._floor_init = True
        else:
            self.noise_floor_db += self._alpha * (db - self.noise_floor_db)

    def _open(self) -> Segment:
        onset_t = self.t - self._onset_run * self.frame_sec
        seg = Segment(seg_id=self._next_id, t_start=onset_t)
        self._next_id += 1
        for frame, prob, db in self._pre:
            seg.audio.append(frame)
            seg.probs.append(prob)
            seg.dbs.append(db)
        self._pre.clear()
        self.current = seg
        self.in_speech = True
        self._silence_run = 0
        self._onset_run = 0
        return seg

    def _close(self, cut_by_tts: bool = False) -> Segment:
        seg = self.current
        assert seg is not None
        seg.t_end = self.t - max(0, self._silence_run - self._pad) * self.frame_sec
        seg.cut_by_tts = cut_by_tts
        self.current = None
        self.in_speech = False
        self._silence_run = 0
        self._onset_run = 0
        return seg
