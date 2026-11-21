"""Streaming wrapper around DeepFilterNet3 for a continuous 16 kHz stream.

The old ``noise_cancellation`` node called ``enhance()`` on every 64 ms chunk
independently and peak-normalised each chunk. The ReSpeaker noise study found
that significantly worse than continuous processing (+3.5 to +6.6 WER points)
because DFN's recurrent state restarts every chunk, and the per-chunk gain
pumps the noise floor.

Here every block of ``block_sec`` is enhanced together with ``context_sec`` of
the audio before it, so the recurrent state has warmed up when it reaches the
new samples; only the new samples are kept (minus a short look-ahead hold-back,
because DFN looks ~2 frames ahead), and consecutive blocks are cross-faded.
The level is normalised with a slow peak tracker instead of per chunk.

The DFN call is injected (``enhance_fn``) so the wrapper is testable without
the model: ``enhance_fn(audio_48k: np.ndarray float32) -> np.ndarray``.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from scipy.signal import resample_poly

SR = 16000
UP = 3  # DFN3 runs at 48 kHz


@dataclass
class StreamingConfig:
    block_sec: float = 0.256
    context_sec: float = 1.0
    lookahead_sec: float = 0.032
    xfade_sec: float = 0.008
    peak_tau_sec: float = 5.0
    min_peak: float = 1e-3


class StreamingDenoiser:
    def __init__(
        self,
        enhance_fn: Callable[[np.ndarray], np.ndarray],
        cfg: StreamingConfig | None = None,
    ):
        self.fn = enhance_fn
        self.cfg = c = cfg or StreamingConfig()
        self.block = int(c.block_sec * SR)
        self.ctx = int(c.context_sec * SR)
        self.la = int(c.lookahead_sec * SR)
        self.xf = int(c.xfade_sec * SR)
        self._hist = np.zeros(0, dtype=np.float32)  # context + pending input
        self._new = 0  # samples in _hist not yet output
        self._tail = None  # previous run's output just past its block (cross-fade)
        self._peak = c.min_peak
        self._decay = np.exp(-self.block / (c.peak_tau_sec * SR))
        self._fade = (
            np.linspace(0.0, 1.0, self.xf, dtype=np.float32) if self.xf else None
        )

    @property
    def latency_sec(self) -> float:
        return (self.block + self.la) / SR

    def process(self, x: np.ndarray) -> np.ndarray:
        """Feed float32 samples in [-1, 1]; returns enhanced samples ready so far."""
        self._hist = np.concatenate([self._hist, x.astype(np.float32)])
        self._new += x.shape[0]
        out = []
        # need the block plus look-ahead beyond it
        while self._new >= self.block + self.la:
            out.append(self._run())
            self._new -= self.block
        keep = self.ctx + self._new + self.la + self.xf
        if self._hist.shape[0] > keep:
            self._hist = self._hist[-keep:]
        return np.concatenate(out) if out else np.zeros(0, dtype=np.float32)

    def _run(self) -> np.ndarray:
        # window: [context | (xfade) block | look-ahead], ending at the newest sample
        end = self._hist.shape[0] - (self._new - self.block - self.la)
        start = max(0, end - self.la - self.block - self.xf - self.ctx)
        seg = self._hist[start:end]
        peak = float(np.max(np.abs(seg[-(self.block + self.la) :]), initial=0.0))
        self._peak = max(self.cfg.min_peak, peak, self._peak * self._decay)
        y48 = self.fn(resample_poly(seg / self._peak, UP, 1).astype(np.float32))
        y = resample_poly(np.asarray(y48, dtype=np.float32), 1, UP).astype(np.float32)
        y = y[: seg.shape[0]] * self._peak
        if not np.all(np.isfinite(y)):
            y = seg.copy()
        stop = y.shape[0] - self.la
        blk = y[stop - self.block : stop].copy()
        if self.xf:
            # the previous run also produced these first xf samples (from its
            # look-ahead region): cross-fade them to hide the seam
            if self._tail is not None:
                blk[: self.xf] = (
                    self._tail * (1 - self._fade) + blk[: self.xf] * self._fade
                )
            self._tail = y[stop : stop + self.xf].copy()
        return blk


def make_dfn_enhance(model, df_state, df_module, atten_lim_db: float | None):
    """enhance_fn for DeepFilterNet3 (``df`` package). ``atten_lim_db``: max
    attenuation; None/0 means unlimited in DF -- avoid, it hurts ASR."""
    import torch

    lim = atten_lim_db if atten_lim_db else None

    def fn(audio48: np.ndarray) -> np.ndarray:
        t = torch.from_numpy(audio48).unsqueeze(0)  # CPU tensor; DF moves it
        with torch.no_grad():
            y = df_module.enhance(model, df_state, t, atten_lim_db=lim)
        return y.cpu().numpy().squeeze().astype(np.float32)

    return fn
