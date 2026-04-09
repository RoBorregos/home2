#!/usr/bin/env python3
"""
vad_benchmark.py — Compare Pitch-Tracking VAD vs Silero VAD

Validates that the autocorrelation-based pitch-tracking VAD reduces false
positives on background noise scenarios where Silero (used internally by
faster-whisper) lacks pitch awareness and can confuse noise with speech.

Usage:
    python3 vad_benchmark.py
    python3 vad_benchmark.py --plot
    python3 vad_benchmark.py --audio-dir ./benchmark_audio --save-csv results.csv

Real audio files (optional):
    Put .wav files at 16 kHz mono inside --audio-dir.
    Prefix filename with:
        speech_*.wav  → labelled as speech
        noise_*.wav   → labelled as non-speech
        mixed_*.wav   → labelled as speech (speech+noise mixture)
"""

from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import scipy.fft
from scipy.spatial.distance import cosine as cosine_distance

# ── Optional imports ──────────────────────────────────────────────────────────

try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    import soundfile as sf

    HAS_SOUNDFILE = True
except ImportError:
    HAS_SOUNDFILE = False

# Silero VAD — loaded via onnxruntime from the local ONNX asset.
# Search order:
#   1. faster_whisper built-in silero_vad_v6.onnx  (venv install)
#   2. speech package assets/silero_vad.onnx        (always present in this repo)
HAS_SILERO = False
_silero_onnx_model = None
_SILERO_WARN = ""


def _load_silero_onnx():
    import onnxruntime

    # Option 1: faster_whisper ships silero_vad_v6.onnx
    try:
        from faster_whisper.vad import get_vad_model as _fw_get

        return _fw_get()
    except Exception:
        pass

    # Option 2: load from the speech package assets sitting next to this script
    _here = os.path.dirname(os.path.abspath(__file__))
    _asset = os.path.normpath(os.path.join(_here, "..", "assets", "silero_vad.onnx"))
    if not os.path.isfile(_asset):
        raise FileNotFoundError(f"silero_vad.onnx not found at {_asset}")

    opts = onnxruntime.SessionOptions()
    opts.inter_op_num_threads = 1
    opts.intra_op_num_threads = 1
    opts.log_severity_level = 4
    session = onnxruntime.InferenceSession(
        _asset, providers=["CPUExecutionProvider"], sess_options=opts
    )

    # Wrap in the same interface as faster_whisper's SileroVADModel
    class _OnnxSilero:
        def __init__(self, sess):
            self.session = sess

        def __call__(
            self,
            audio: np.ndarray,
            num_samples: int = 512,
            context_size_samples: int = 64,
        ):
            assert audio.ndim == 1
            assert audio.shape[0] % num_samples == 0
            # Detect model version from required inputs
            input_names = {inp.name for inp in self.session.get_inputs()}
            has_sr = "sr" in input_names

            h_size = 64 if has_sr else 128
            h = np.zeros((2, 1, h_size), dtype="float32")
            c = np.zeros((2, 1, h_size), dtype="float32")
            sr = np.array(SAMPLE_RATE, dtype=np.int64)
            outputs = []
            for start in range(0, audio.shape[0], num_samples):
                chunk = audio[start : start + num_samples].reshape(1, -1)
                feed = {"input": chunk, "h": h, "c": c}
                if has_sr:
                    feed["sr"] = sr
                out, h, c = self.session.run(None, feed)
                outputs.append(out)
            return np.concatenate(outputs, axis=0)

    return _OnnxSilero(session)


try:
    _silero_onnx_model = _load_silero_onnx()
    HAS_SILERO = True
except Exception as _exc:
    _SILERO_WARN = str(_exc)

# ── Constants ─────────────────────────────────────────────────────────────────

SAMPLE_RATE = 16000
CHUNK = 1024  # samples per VAD frame (~64 ms)

VOWEL_FREQ_LOW = 85  # Hz  — fundamental pitch lower bound
VOWEL_FREQ_HIGH = 255  # Hz  — fundamental pitch upper bound

# PitchVAD defaults — match voice_detection.yaml
ENERGY_THRESHOLD = 600.0
CORRELATION_THRESHOLD = 0.62
SIMILARITY_THRESHOLD = 0.70

# Silero confidence gate
SILERO_THRESHOLD = 0.5


# ── Pitch-Tracking VAD ────────────────────────────────────────────────────────


class PitchTrackingVAD:
    """
    Mirrors the production VoiceDetection node (scripts/voice_detection.py).

    Three cascaded tests per chunk:
      1. RMS energy gate          — rejects silence and very faint sounds.
      2. Autocorrelation pitch    — rejects non-vocal noise (no pitch in 85-255 Hz).
      3. MFCC cosine similarity   — locks onto the primary speaker; rejects others.
    """

    def __init__(
        self,
        sample_rate: int = SAMPLE_RATE,
        energy_threshold: float = ENERGY_THRESHOLD,
        correlation_threshold: float = CORRELATION_THRESHOLD,
        similarity_threshold: float = SIMILARITY_THRESHOLD,
    ) -> None:
        self.sample_rate = sample_rate
        self.energy_threshold = energy_threshold
        self.correlation_threshold = correlation_threshold
        self.similarity_threshold = similarity_threshold
        self._low_lag = int(sample_rate / VOWEL_FREQ_HIGH)
        self._high_lag = int(sample_rate / VOWEL_FREQ_LOW)
        self._target_mfcc: Optional[np.ndarray] = None
        self._is_locked = False

    def reset(self) -> None:
        self._target_mfcc = None
        self._is_locked = False

    # ── internal helpers (identical to production) ─────────────────────────

    def _get_mfcc(self, audio_f: np.ndarray) -> np.ndarray:
        n_mfcc, n_fft, n_mels, hop = 13, 512, 40, 160
        num_frames = max(1, (len(audio_f) - n_fft) // hop + 1)
        frames = (
            np.stack(
                [
                    audio_f[i * hop : i * hop + n_fft] * np.hanning(n_fft)
                    for i in range(num_frames)
                    if i * hop + n_fft <= len(audio_f)
                ]
            )
            if num_frames > 0
            else np.zeros((1, n_fft))
        )
        power = np.abs(np.fft.rfft(frames, n=n_fft)) ** 2
        fmin, fmax = 0.0, self.sample_rate / 2.0
        mel_pts = np.linspace(
            2595 * np.log10(1 + fmin / 700),
            2595 * np.log10(1 + fmax / 700),
            n_mels + 2,
        )
        hz_pts = 700 * (10 ** (mel_pts / 2595) - 1)
        bins = np.floor((n_fft + 1) * hz_pts / self.sample_rate).astype(int)
        fb = np.zeros((n_mels, n_fft // 2 + 1))
        for m in range(1, n_mels + 1):
            lo, mid, hi = bins[m - 1], bins[m], bins[m + 1]
            if mid > lo:
                fb[m - 1, lo:mid] = (np.arange(lo, mid) - lo) / (mid - lo)
            if hi > mid:
                fb[m - 1, mid:hi] = (hi - np.arange(mid, hi)) / (hi - mid)
        mel_e = np.maximum(np.dot(power, fb.T), np.finfo(float).eps)
        mfccs = scipy.fft.dct(np.log(mel_e), type=2, norm="ortho", axis=-1)[:, :n_mfcc]
        return np.mean(mfccs, axis=0)

    def _is_periodic(self, audio_f: np.ndarray) -> bool:
        corr = np.correlate(audio_f, audio_f, mode="full")
        corr = corr[len(corr) // 2 :]
        if len(corr) <= self._high_lag or corr[0] == 0:
            return False
        peak = np.max(corr[self._low_lag : self._high_lag])
        return (peak / corr[0]) > self.correlation_threshold

    # ── public interface ───────────────────────────────────────────────────

    def process_chunk(self, chunk: np.ndarray) -> Tuple[bool, str]:
        """Return (is_speech, reason_label) for a single audio chunk."""
        audio_f = chunk.astype(np.float32)
        rms = np.sqrt(np.mean(audio_f**2))

        if rms < self.energy_threshold:
            return False, "silence"

        if not self._is_periodic(audio_f):
            return False, "non-vocal"

        mfcc = self._get_mfcc(audio_f)
        if not self._is_locked:
            self._target_mfcc = mfcc
            self._is_locked = True
            return True, "lock-on"

        sim = 1.0 - cosine_distance(mfcc, self._target_mfcc)
        if sim >= self.similarity_threshold:
            return True, f"speech(sim={sim:.2f})"
        return False, f"other-speaker(sim={sim:.2f})"


# ── Silero VAD wrapper ────────────────────────────────────────────────────────

# Silero's ONNX model (from faster_whisper) processes audio in 512-sample windows.
# CHUNK=1024 maps to exactly 2 Silero windows; we take the max confidence of both.
_SILERO_WIN = 512


class SileroVAD:
    def __init__(self, threshold: float = SILERO_THRESHOLD) -> None:
        if not HAS_SILERO or _silero_onnx_model is None:
            raise RuntimeError("Silero VAD not available")
        self.model = _silero_onnx_model
        self.threshold = threshold
        self._probs: Optional[np.ndarray] = None  # computed once per scenario

    def reset(self) -> None:
        self._probs = None

    def run_on_audio(self, audio: np.ndarray) -> None:
        """Pre-compute per-window probabilities for a full audio array (int16)."""
        audio_f = audio.astype(np.float32) / 32768.0
        # Pad to multiple of 512 as required by SileroVADModel.__call__
        pad = (_SILERO_WIN - len(audio_f) % _SILERO_WIN) % _SILERO_WIN
        padded = np.pad(audio_f, (0, pad))
        self._probs = self.model(padded).flatten()  # one prob per 512-sample window

    def chunk_pred(self, chunk_idx: int) -> Tuple[bool, str]:
        """Return (is_speech, label) for the chunk at chunk_idx (each chunk = CHUNK samples)."""
        if self._probs is None:
            return False, "no-probs"
        # Each CHUNK (1024) = 2 Silero windows of 512
        w0 = chunk_idx * (CHUNK // _SILERO_WIN)
        w1 = w0 + (CHUNK // _SILERO_WIN) - 1
        w1 = min(w1, len(self._probs) - 1)
        conf = float(np.max(self._probs[w0 : w1 + 1]))
        return conf >= self.threshold, f"conf={conf:.2f}"


# ── Synthetic audio generation ────────────────────────────────────────────────


def _to_int16(audio_f: np.ndarray, amplitude: float = 20_000) -> np.ndarray:
    mx = np.max(np.abs(audio_f))
    if mx > 0:
        audio_f = audio_f / mx * amplitude
    return audio_f.astype(np.int16)


def gen_voiced_speech(
    duration_s: float, f0: float = 150.0, n_harmonics: int = 6
) -> np.ndarray:
    """
    Simulate voiced speech as a harmonic complex with F0 in vocal range
    plus a slow AM envelope (~4 Hz) to mimic syllable rhythm.
    Autocorrelation will show a clear peak at lag ≈ RATE/F0 (within 85-255 Hz window).
    """
    t = np.linspace(0, duration_s, int(SAMPLE_RATE * duration_s), endpoint=False)
    signal = sum(
        (1.0 / (k + 1)) * np.sin(2 * np.pi * f0 * (k + 1) * t)
        for k in range(n_harmonics)
    )
    env = 0.6 + 0.4 * np.sin(2 * np.pi * 4.0 * t)  # syllable-rate envelope
    return _to_int16(signal * env)


def gen_white_noise(duration_s: float, seed: int = 42) -> np.ndarray:
    """Broadband Gaussian noise — no pitch structure."""
    rng = np.random.default_rng(seed)
    return _to_int16(rng.standard_normal(int(SAMPLE_RATE * duration_s)))


def gen_pink_noise(duration_s: float, seed: int = 42) -> np.ndarray:
    """
    Pink (1/f) noise — models fan / HVAC background noise.
    Shaped in frequency domain; no periodic pitch structure in vocal range.
    """
    n = int(SAMPLE_RATE * duration_s)
    rng = np.random.default_rng(seed)
    white = rng.standard_normal(n)
    freqs = np.fft.rfftfreq(n)
    freqs[0] = 1e-6
    spectrum = np.fft.rfft(white) / np.sqrt(freqs)
    spectrum[0] = 0
    return _to_int16(np.fft.irfft(spectrum, n=n))


def gen_hum_noise(duration_s: float, freq: float = 60.0) -> np.ndarray:
    """
    Power-line / AC hum — periodic, but F0 is BELOW vocal range (85 Hz floor).
    Autocorrelation lag for 60 Hz = 16000/60 ≈ 267 samples > high_lag (188),
    so _is_periodic() correctly returns False.
    """
    t = np.linspace(0, duration_s, int(SAMPLE_RATE * duration_s), endpoint=False)
    return _to_int16(np.sin(2 * np.pi * freq * t))


def gen_machinery_noise(duration_s: float, seed: int = 42) -> np.ndarray:
    """
    Broadband noise with a strong low-frequency component (motor rumble).
    Models air conditioning units or ventilation fans.
    """
    n = int(SAMPLE_RATE * duration_s)
    pink = gen_pink_noise(duration_s, seed).astype(np.float32)
    t = np.linspace(0, duration_s, n, endpoint=False)
    # Add strong 40 Hz rumble (below vocal pitch range)
    rumble = np.sin(2 * np.pi * 40.0 * t) * 0.6
    return _to_int16(pink / 20_000 + rumble)


def gen_impact_noise(
    duration_s: float, rate_hz: float = 3.0, seed: int = 42
) -> np.ndarray:
    """
    Rhythmic percussive impacts (keyboard, footsteps).
    Periodic cadence but NOT a pitched sound in the vocal range.
    """
    n = int(SAMPLE_RATE * duration_s)
    rng = np.random.default_rng(seed)
    signal = np.zeros(n, dtype=np.float32)
    period = int(SAMPLE_RATE / rate_hz)
    for start in range(0, n, period):
        burst_len = min(int(0.012 * SAMPLE_RATE), n - start)
        decay = np.exp(-np.linspace(0, 6, burst_len))
        signal[start : start + burst_len] += rng.standard_normal(burst_len) * decay
    return _to_int16(signal)


def mix(speech: np.ndarray, noise: np.ndarray, snr_db: float) -> np.ndarray:
    """Mix speech and noise at the specified SNR (dB)."""
    s = speech.astype(np.float32)
    n = noise.astype(np.float32)
    min_len = min(len(s), len(n))
    s, n = s[:min_len], n[:min_len]
    rms_s = np.sqrt(np.mean(s**2)) + 1e-9
    rms_n = np.sqrt(np.mean(n**2)) + 1e-9
    scale = rms_s / (rms_n * 10 ** (snr_db / 20.0))
    return _to_int16(s + n * scale)


# ── Benchmark scenario definitions ───────────────────────────────────────────


@dataclass
class Scenario:
    name: str
    audio: np.ndarray
    ground_truth: bool  # True = speech present
    category: str  # "speech" | "noise" | "mixed"
    description: str = ""


def build_scenarios() -> List[Scenario]:
    dur = 2.0
    speech_150 = gen_voiced_speech(dur, f0=150.0)
    speech_120 = gen_voiced_speech(dur, f0=120.0)
    speech_190 = gen_voiced_speech(dur, f0=190.0)
    white = gen_white_noise(dur)
    pink = gen_pink_noise(dur)
    hum = gen_hum_noise(dur, 60.0)
    machinery = gen_machinery_noise(dur)
    impact = gen_impact_noise(dur)

    return [
        # ── Pure noise — expected: no speech detected ─────────────────────
        Scenario(
            "white_noise",
            white,
            False,
            "noise",
            "White Gaussian noise — no pitch structure",
        ),
        Scenario(
            "pink_noise_fan",
            pink,
            False,
            "noise",
            "Pink (1/f) noise — models fan / HVAC",
        ),
        Scenario(
            "hum_60hz",
            hum,
            False,
            "noise",
            "60 Hz power-line hum — periodic but below vocal range",
        ),
        Scenario(
            "machinery_rumble",
            machinery,
            False,
            "noise",
            "Motor rumble + broadband noise — no vocal pitch",
        ),
        Scenario(
            "impact_keyboard",
            impact,
            False,
            "noise",
            "Rhythmic percussive impacts — periodic but not pitched",
        ),
        # ── Clean speech — expected: speech detected ──────────────────────
        Scenario(
            "clean_speech_150",
            speech_150,
            True,
            "speech",
            "Voiced speech @ 150 Hz F0 (mid-range)",
        ),
        Scenario(
            "clean_speech_120",
            speech_120,
            True,
            "speech",
            "Voiced speech @ 120 Hz F0 (male, low)",
        ),
        Scenario(
            "clean_speech_190",
            speech_190,
            True,
            "speech",
            "Voiced speech @ 190 Hz F0 (female, high)",
        ),
        # ── Speech + noise mixtures — expected: speech detected ───────────
        Scenario(
            "speech_white_9db",
            mix(speech_150, white, 9.0),
            True,
            "mixed",
            "Speech + white noise @ 9 dB SNR",
        ),
        Scenario(
            "speech_white_3db",
            mix(speech_150, white, 3.0),
            True,
            "mixed",
            "Speech + white noise @ 3 dB SNR (challenging)",
        ),
        Scenario(
            "speech_pink_6db",
            mix(speech_150, pink, 6.0),
            True,
            "mixed",
            "Speech + fan noise @ 6 dB SNR",
        ),
        Scenario(
            "speech_pink_0db",
            mix(speech_150, pink, 0.0),
            True,
            "mixed",
            "Speech + fan noise @ 0 dB SNR (very noisy)",
        ),
        Scenario(
            "speech_hum_6db",
            mix(speech_150, hum, 6.0),
            True,
            "mixed",
            "Speech + 60 Hz hum @ 6 dB SNR",
        ),
        Scenario(
            "speech_machine_6db",
            mix(speech_150, machinery, 6.0),
            True,
            "mixed",
            "Speech + motor rumble @ 6 dB SNR",
        ),
    ]


# ── Evaluation ────────────────────────────────────────────────────────────────


@dataclass
class ScenarioResult:
    scenario: Scenario
    pitch_pred: bool
    silero_pred: Optional[bool]
    pitch_time_ms: float
    silero_time_ms: Optional[float]
    pitch_chunk_preds: List[bool] = field(default_factory=list)
    silero_chunk_preds: List[bool] = field(default_factory=list)
    pitch_chunk_reasons: List[str] = field(default_factory=list)

    @property
    def pitch_correct(self) -> bool:
        return self.pitch_pred == self.scenario.ground_truth

    @property
    def silero_correct(self) -> Optional[bool]:
        if self.silero_pred is None:
            return None
        return self.silero_pred == self.scenario.ground_truth


def _majority(preds: List[bool]) -> bool:
    return (sum(preds) / max(len(preds), 1)) >= 0.5


def run_scenario(
    scenario: Scenario,
    pitch_vad: PitchTrackingVAD,
    silero_vad: Optional[SileroVAD],
) -> ScenarioResult:
    audio = scenario.audio
    chunks = [audio[i : i + CHUNK] for i in range(0, len(audio) - CHUNK + 1, CHUNK)]

    pitch_vad.reset()
    pitch_preds: List[bool] = []
    pitch_reasons: List[str] = []
    t0 = time.perf_counter()
    for chunk in chunks:
        pred, reason = pitch_vad.process_chunk(chunk)
        pitch_preds.append(pred)
        pitch_reasons.append(reason)
    pitch_time = (time.perf_counter() - t0) * 1000.0

    silero_preds: Optional[List[bool]] = None
    silero_time: Optional[float] = None
    if silero_vad is not None:
        silero_vad.reset()
        t0 = time.perf_counter()
        silero_vad.run_on_audio(audio)
        silero_preds = [silero_vad.chunk_pred(i)[0] for i in range(len(chunks))]
        silero_time = (time.perf_counter() - t0) * 1000.0

    return ScenarioResult(
        scenario=scenario,
        pitch_pred=_majority(pitch_preds),
        silero_pred=_majority(silero_preds) if silero_preds is not None else None,
        pitch_time_ms=pitch_time,
        silero_time_ms=silero_time,
        pitch_chunk_preds=pitch_preds,
        silero_chunk_preds=silero_preds or [],
        pitch_chunk_reasons=pitch_reasons,
    )


# ── Metrics ───────────────────────────────────────────────────────────────────


def _metrics(preds: List[bool], truths: List[bool]) -> Dict:
    tp = sum(p and t for p, t in zip(preds, truths))
    fp = sum(p and not t for p, t in zip(preds, truths))
    tn = sum(not p and not t for p, t in zip(preds, truths))
    fn = sum(not p and t for p, t in zip(preds, truths))
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1 = (
        (2 * precision * recall / (precision + recall))
        if (precision + recall) > 0
        else 0.0
    )
    fpr = fp / (fp + tn) if (fp + tn) > 0 else 0.0
    acc = (tp + tn) / max(tp + fp + tn + fn, 1)
    return dict(
        tp=tp,
        fp=fp,
        tn=tn,
        fn=fn,
        precision=precision,
        recall=recall,
        f1=f1,
        fpr=fpr,
        acc=acc,
    )


# ── Console report ────────────────────────────────────────────────────────────

W = 118
TICK = "✓"
CROSS = "✗"


def _pred_label(pred: bool, correct: bool) -> str:
    mark = TICK if correct else CROSS
    return f"{mark} {'SPEECH' if pred else 'NOISE '}"


def print_report(results: List[ScenarioResult], include_silero: bool) -> None:
    truths = [r.scenario.ground_truth for r in results]
    pitch_preds = [r.pitch_pred for r in results]

    print("\n" + "=" * W)
    print(" VAD BENCHMARK — Pitch-Tracking VAD  vs  Silero VAD ".center(W, "="))
    print("=" * W)

    # ── Per-scenario table ────────────────────────────────────────────────
    name_w, gt_w, desc_w, vad_w = 26, 7, 52, 14
    hdr = (
        f"  {'Scenario':<{name_w}} {'GT':^{gt_w}} {'Description':<{desc_w}}"
        f" {'PitchVAD':^{vad_w}}"
    )
    if include_silero:
        hdr += f" {'SileroVAD':^{vad_w}}"

    category_order = ["noise", "speech", "mixed"]
    for cat in category_order:
        cat_results = [r for r in results if r.scenario.category == cat]
        if not cat_results:
            continue
        print(f"\n  ── {cat.upper()} scenarios " + "─" * (W - 20))
        print(hdr)
        print("  " + "-" * (W - 2))
        for r in cat_results:
            gt_label = "SPEECH" if r.scenario.ground_truth else "NOISE "
            p_label = _pred_label(r.pitch_pred, r.pitch_correct)
            row = (
                f"  {r.scenario.name:<{name_w}} {gt_label:^{gt_w}} "
                f"{r.scenario.description:<{desc_w}} {p_label:^{vad_w}}"
            )
            if include_silero and r.silero_pred is not None:
                s_label = _pred_label(r.silero_pred, r.silero_correct)  # type: ignore[arg-type]
                row += f" {s_label:^{vad_w}}"
            print(row)

    # ── Aggregate metrics ─────────────────────────────────────────────────
    print("\n" + "─" * W)
    print(" AGGREGATE METRICS (majority vote across chunks per scenario) ".center(W))
    print("─" * W)

    def _fmt(m: Dict) -> str:
        return (
            f"Acc={m['acc']:.0%}  Precision={m['precision']:.0%}  "
            f"Recall={m['recall']:.0%}  F1={m['f1']:.0%}  FPR={m['fpr']:.0%}  "
            f"[TP={m['tp']} FP={m['fp']} TN={m['tn']} FN={m['fn']}]"
        )

    pm = _metrics(pitch_preds, truths)
    print(f"\n  PitchTrackingVAD  →  {_fmt(pm)}")
    if include_silero:
        silero_preds = [r.silero_pred for r in results]
        sm = _metrics(silero_preds, truths)  # type: ignore[arg-type]
        print(f"  SileroVAD         →  {_fmt(sm)}")

    # ── False-positive spotlight (noise-only scenarios) ───────────────────
    noise_results = [r for r in results if r.scenario.category == "noise"]
    if noise_results:
        print(f"\n  {'─'*60}")
        print("  FALSE POSITIVES on pure-noise scenarios (key metric):")
        p_fp = sum(r.pitch_pred for r in noise_results)
        print(
            f"    PitchTrackingVAD:  {p_fp}/{len(noise_results)} noise chunks "
            f"incorrectly labelled as speech"
        )
        if include_silero:
            s_fp = sum(
                r.silero_pred for r in noise_results if r.silero_pred is not None
            )
            print(
                f"    SileroVAD:         {s_fp}/{len(noise_results)} noise chunks "
                f"incorrectly labelled as speech"
            )
        print(f"  {'─'*60}")

    # ── Timing ────────────────────────────────────────────────────────────
    dur_ms = len(results[0].scenario.audio) / SAMPLE_RATE * 1000
    avg_pitch_ms = float(np.mean([r.pitch_time_ms for r in results]))
    print(f"\n  Audio duration per scenario:  {dur_ms:.0f} ms")
    print("  Avg inference time:")
    rtf_pitch = avg_pitch_ms / dur_ms
    print(f"    PitchTrackingVAD:  {avg_pitch_ms:.1f} ms  (RTF={rtf_pitch:.3f})")
    if include_silero:
        silero_times = [
            r.silero_time_ms for r in results if r.silero_time_ms is not None
        ]
        avg_silero_ms = float(np.mean(silero_times))
        rtf_silero = avg_silero_ms / dur_ms
        print(f"    SileroVAD:         {avg_silero_ms:.1f} ms  (RTF={rtf_silero:.3f})")

    print("\n" + "=" * W + "\n")


# ── Optional plot ─────────────────────────────────────────────────────────────


def plot_results(results: List[ScenarioResult], include_silero: bool) -> None:
    if not HAS_MATPLOTLIB:
        print(
            "[WARNING] matplotlib not available — skipping plot. pip install matplotlib"
        )
        return

    n = len(results)
    fig, axes = plt.subplots(n, 1, figsize=(15, 2.2 * n), sharex=False)
    if n == 1:
        axes = [axes]

    for ax, r in zip(axes, results):
        t = np.arange(len(r.scenario.audio)) / SAMPLE_RATE
        ax.plot(
            t,
            r.scenario.audio,
            color="steelblue",
            alpha=0.45,
            linewidth=0.5,
            label="audio",
        )

        for i, pred in enumerate(r.pitch_chunk_preds):
            if pred:
                ax.axvspan(
                    i * CHUNK / SAMPLE_RATE,
                    (i + 1) * CHUNK / SAMPLE_RATE,
                    alpha=0.35,
                    color="limegreen",
                    lw=0,
                )
        if include_silero and r.silero_chunk_preds:
            for i, pred in enumerate(r.silero_chunk_preds):
                if pred:
                    ax.axvspan(
                        i * CHUNK / SAMPLE_RATE,
                        (i + 1) * CHUNK / SAMPLE_RATE,
                        alpha=0.20,
                        color="tomato",
                        lw=0,
                    )

        title_color = "darkgreen" if r.scenario.ground_truth else "firebrick"
        extra = ""
        if include_silero and r.silero_pred is not None:
            silero_ok = "✓" if r.silero_correct else "✗"
            extra = f"  |  Silero {silero_ok}"
        ax.set_title(
            f"{r.scenario.name}  [GT={'SPEECH' if r.scenario.ground_truth else 'NOISE'}]"
            f"  PitchVAD {'✓' if r.pitch_correct else '✗'}{extra}",
            fontsize=8.5,
            color=title_color,
            pad=3,
        )
        ax.set_ylabel("Amp", fontsize=7)
        ax.tick_params(labelsize=6)
        ax.set_xlim(0, t[-1])

    axes[-1].set_xlabel("Time (s)", fontsize=8)

    legend_handles = [
        Patch(facecolor="limegreen", alpha=0.5, label="PitchVAD — detected speech")
    ]
    if include_silero:
        legend_handles.append(
            Patch(facecolor="tomato", alpha=0.4, label="SileroVAD — detected speech")
        )
    fig.legend(handles=legend_handles, loc="upper right", fontsize=8, framealpha=0.8)
    plt.suptitle(
        "VAD Benchmark — chunk-level detections per scenario", fontsize=11, y=1.002
    )
    plt.tight_layout(h_pad=0.6)

    out_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "vad_benchmark_plot.png"
    )
    plt.savefig(out_path, dpi=130, bbox_inches="tight")
    print(f"  [plot] Saved to: {out_path}")
    plt.show()


# ── Real audio loading ────────────────────────────────────────────────────────


def load_wav_scenarios(audio_dir: str) -> List[Scenario]:
    """
    Load mono 16 kHz .wav files from audio_dir.

    Naming convention:
        speech_*.wav  → ground_truth = True   (category: speech)
        noise_*.wav   → ground_truth = False  (category: noise)
        mixed_*.wav   → ground_truth = True   (category: mixed)
    """
    if not HAS_SOUNDFILE:
        print(
            "[WARNING] soundfile not installed — skipping real audio. pip install soundfile"
        )
        return []

    scenarios: List[Scenario] = []
    for fname in sorted(os.listdir(audio_dir)):
        if not fname.lower().endswith(".wav"):
            continue
        fpath = os.path.join(audio_dir, fname)
        try:
            audio, sr = sf.read(fpath, dtype="int16")
        except Exception as e:
            print(f"[WARNING] Could not read {fname}: {e}")
            continue
        if sr != SAMPLE_RATE:
            print(f"[WARNING] {fname}: sample rate {sr} ≠ {SAMPLE_RATE} — skipping")
            continue
        if audio.ndim > 1:
            audio = audio[:, 0]  # take first channel

        stem = fname.rsplit(".", 1)[0]
        if stem.startswith("speech"):
            gt, cat = True, "speech"
        elif stem.startswith("noise"):
            gt, cat = False, "noise"
        elif stem.startswith("mixed"):
            gt, cat = True, "mixed"
        else:
            print(f"[WARNING] {fname}: no speech/noise/mixed prefix — skipping")
            continue

        scenarios.append(Scenario(stem, audio, gt, cat, f"Real audio: {fname}"))

    return scenarios


# ── CSV export ────────────────────────────────────────────────────────────────


def save_csv(results: List[ScenarioResult], path: str, include_silero: bool) -> None:
    import csv

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        header = [
            "scenario",
            "category",
            "ground_truth",
            "pitch_pred",
            "pitch_correct",
            "pitch_time_ms",
        ]
        if include_silero:
            header += ["silero_pred", "silero_correct", "silero_time_ms"]
        writer.writerow(header)
        for r in results:
            row = [
                r.scenario.name,
                r.scenario.category,
                r.scenario.ground_truth,
                r.pitch_pred,
                r.pitch_correct,
                f"{r.pitch_time_ms:.1f}",
            ]
            if include_silero:
                row += [r.silero_pred, r.silero_correct, f"{r.silero_time_ms:.1f}"]
            writer.writerow(row)
    print(f"  [csv] Saved to: {path}")


# ── CLI ───────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark: Pitch-Tracking VAD vs Silero VAD",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--audio-dir",
        default=None,
        help="Directory with labelled .wav files (speech_*, noise_*, mixed_*)",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Render chunk-level detection plot (requires matplotlib)",
    )
    parser.add_argument(
        "--save-csv",
        default=None,
        metavar="FILE",
        help="Export per-scenario results to CSV",
    )
    parser.add_argument("--energy-threshold", type=float, default=ENERGY_THRESHOLD)
    parser.add_argument("--corr-threshold", type=float, default=CORRELATION_THRESHOLD)
    parser.add_argument("--sim-threshold", type=float, default=SIMILARITY_THRESHOLD)
    parser.add_argument("--silero-threshold", type=float, default=SILERO_THRESHOLD)
    args = parser.parse_args()

    pitch_vad = PitchTrackingVAD(
        energy_threshold=args.energy_threshold,
        correlation_threshold=args.corr_threshold,
        similarity_threshold=args.sim_threshold,
    )

    silero_vad: Optional[SileroVAD] = None
    if HAS_SILERO:
        try:
            silero_vad = SileroVAD(threshold=args.silero_threshold)
            print("[INFO] Silero VAD loaded successfully.\n")
        except Exception as exc:
            print(f"[WARNING] Silero VAD init failed: {exc}\n")
    else:
        print(f"[WARNING] Silero VAD not available: {_SILERO_WARN}")
        print("[INFO]    Make sure onnxruntime is installed: pip install onnxruntime\n")

    # Build scenario list
    scenarios = build_scenarios()
    if args.audio_dir:
        extra = load_wav_scenarios(args.audio_dir)
        if extra:
            print(
                f"[INFO] Loaded {len(extra)} real audio scenarios from '{args.audio_dir}'\n"
            )
        scenarios.extend(extra)

    print(
        f"[INFO] Running {len(scenarios)} scenarios "
        f"({'with' if silero_vad else 'without'} Silero comparison) ...\n"
    )

    results = [run_scenario(s, pitch_vad, silero_vad) for s in scenarios]

    print_report(results, include_silero=silero_vad is not None)

    if args.save_csv:
        save_csv(results, args.save_csv, include_silero=silero_vad is not None)

    if args.plot:
        plot_results(results, include_silero=silero_vad is not None)


if __name__ == "__main__":
    main()
