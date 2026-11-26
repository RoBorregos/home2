"""Speaker identification utilities (issue #1459), used by
``scripts/speaker_verification.py``. Pure python (no ROS):

- fbank features (Kaldi compatible, numpy) and the ONNX speaker embedder
  (WeSpeaker ResNet34-LM by default),
- streaming Silero VAD and the speech segmenter (VAD hysteresis + energy gate),
- enrolled profiles (anchor + adaptive pool, persisted as .npz),
- scoring, TARGET / NOT_TARGET / UNKNOWN / MIXED decisions and enrollment.
"""

from __future__ import annotations

import json
import os
import re
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np


# --------------------------------------------------------------------------
# Features
# Kaldi-compatible log-mel filterbank features.
#
# WeSpeaker models are trained on ``torchaudio.compliance.kaldi.fbank`` with
# 80 mel bins, 25 ms frames, 10 ms shift, hamming window and no dither, applied to
# the waveform scaled to the int16 range, followed by per-utterance mean
# normalisation (CMN). Reproduced here in numpy so the robot does not depend on
# torchaudio.
# --------------------------------------------------------------------------

_EPS = np.finfo(np.float32).eps


def _mel(freq: np.ndarray | float) -> np.ndarray | float:
    return 1127.0 * np.log(1.0 + np.asarray(freq) / 700.0)


def _mel_banks(
    num_bins: int, padded_window: int, sample_rate: int, low_freq: float = 20.0
) -> np.ndarray:
    """Triangular mel filters, same construction as Kaldi / torchaudio."""
    num_fft_bins = padded_window // 2
    nyquist = 0.5 * sample_rate
    fft_bin_width = sample_rate / padded_window
    mel_low = _mel(low_freq)
    mel_high = _mel(nyquist)
    mel_delta = (mel_high - mel_low) / (num_bins + 1)

    bins = np.arange(num_bins, dtype=np.float64)[:, None]
    left = mel_low + bins * mel_delta
    center = left + mel_delta
    right = center + mel_delta

    mel_freqs = _mel(fft_bin_width * np.arange(num_fft_bins, dtype=np.float64))[None, :]
    up = (mel_freqs - left) / (center - left)
    down = (right - mel_freqs) / (right - center)
    banks = np.maximum(0.0, np.minimum(up, down))
    # torchaudio pads one zero column for the Nyquist bin
    return np.pad(banks, ((0, 0), (0, 1))).astype(np.float32)


_BANK_CACHE: dict[tuple[int, int, int], np.ndarray] = {}


def kaldi_fbank(
    wave: np.ndarray,
    sample_rate: int = 16000,
    num_mel_bins: int = 80,
    frame_length_ms: float = 25.0,
    frame_shift_ms: float = 10.0,
) -> np.ndarray:
    """Log-mel filterbank of ``wave`` (float, already scaled to int16 range).

    Returns an array of shape ``[num_frames, num_mel_bins]`` (float32). Empty if
    the waveform is shorter than one frame.
    """
    wave = np.asarray(wave, dtype=np.float64)
    win = int(sample_rate * frame_length_ms / 1000)
    hop = int(sample_rate * frame_shift_ms / 1000)
    if wave.shape[0] < win:
        return np.zeros((0, num_mel_bins), dtype=np.float32)

    num_frames = 1 + (wave.shape[0] - win) // hop
    idx = np.arange(win)[None, :] + hop * np.arange(num_frames)[:, None]
    frames = wave[idx]

    frames = frames - frames.mean(axis=1, keepdims=True)  # remove DC offset
    prev = np.concatenate([frames[:, :1], frames[:, :-1]], axis=1)
    frames = frames - 0.97 * prev  # pre-emphasis
    frames = frames * np.hamming(win)[None, :]

    padded = 1 << (win - 1).bit_length()
    spectrum = np.abs(np.fft.rfft(frames, n=padded, axis=1)) ** 2

    key = (num_mel_bins, padded, sample_rate)
    if key not in _BANK_CACHE:
        _BANK_CACHE[key] = _mel_banks(num_mel_bins, padded, sample_rate)
    mel_energies = spectrum @ _BANK_CACHE[key].T.astype(np.float64)
    return np.log(np.maximum(mel_energies, _EPS)).astype(np.float32)


def speaker_features(wave_f32: np.ndarray, sample_rate: int = 16000) -> np.ndarray:
    """WeSpeaker input features for a float waveform in [-1, 1].

    Returns ``[num_frames, 80]`` with per-utterance mean removed.
    """
    scaled = np.asarray(wave_f32, dtype=np.float32) * 32768.0
    feats = kaldi_fbank(scaled, sample_rate=sample_rate)
    if feats.shape[0] == 0:
        return feats
    return feats - feats.mean(axis=0, keepdims=True)


# --------------------------------------------------------------------------
# Embedder
# ONNX speaker-embedding model wrapper.
# --------------------------------------------------------------------------

DEFAULT_MODEL_FILE = "voxceleb_resnet34_LM.onnx"


def l2_normalize(vec: np.ndarray) -> np.ndarray:
    vec = np.asarray(vec, dtype=np.float32)
    norm = float(np.linalg.norm(vec))
    return vec / norm if norm > 0 else vec


def pick_providers(requested: list[str] | None = None) -> list[str]:
    """Keep only the requested onnxruntime providers that are installed.

    ``requested`` uses short names (``"cuda"``, ``"tensorrt"``, ``"cpu"``) or full
    provider names. CPU is always appended as the last fallback.
    """
    import onnxruntime as ort

    aliases = {
        "cpu": "CPUExecutionProvider",
        "cuda": "CUDAExecutionProvider",
        "tensorrt": "TensorrtExecutionProvider",
        "trt": "TensorrtExecutionProvider",
    }
    available = set(ort.get_available_providers())
    wanted = [aliases.get(p.strip().lower(), p.strip()) for p in (requested or [])]
    providers = [p for p in wanted if p in available]
    if "CPUExecutionProvider" not in providers:
        providers.append("CPUExecutionProvider")
    return providers


class SpeakerEmbedder:
    """Computes L2-normalised speaker embeddings from 16 kHz float audio.

    Works with WeSpeaker ONNX exports (input ``feats [B, T, 80]``, output
    ``embs [B, D]``).
    """

    def __init__(
        self,
        model_path: str,
        providers: list[str] | None = None,
        sample_rate: int = 16000,
        num_threads: int = 2,
    ) -> None:
        import onnxruntime as ort

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = num_threads
        opts.inter_op_num_threads = 1
        opts.log_severity_level = 3
        # Every segment has a different length, so per-shape cuDNN algo search
        # never amortises. Measured on the HRI Orin (JetPack 7.2, ORT 1.24):
        # EXHAUSTIVE p50 56 ms, HEURISTIC 56 ms, DEFAULT 37 ms per 1-6 s segment.
        provider_cfg = [
            (p, {"cudnn_conv_algo_search": "DEFAULT"})
            if p == "CUDAExecutionProvider"
            else p
            for p in pick_providers(providers)
        ]
        self.session = ort.InferenceSession(
            model_path, sess_options=opts, providers=provider_cfg
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.dim = int(self.session.get_outputs()[0].shape[-1])
        self.sample_rate = sample_rate
        self.providers = self.session.get_providers()
        self.last_infer_ms = 0.0

    def embed(self, wave_f32: np.ndarray) -> np.ndarray:
        """Embedding of a mono float waveform in [-1, 1] (at least ~0.5 s)."""
        start = time.perf_counter()
        feats = speaker_features(wave_f32, self.sample_rate)
        if feats.shape[0] < 10:
            raise ValueError("audio too short for a speaker embedding")
        out = self.session.run(
            [self.output_name], {self.input_name: feats[None, :, :]}
        )[0]
        self.last_infer_ms = (time.perf_counter() - start) * 1000.0
        return l2_normalize(out[0])

    def warmup(self) -> None:
        rng = np.random.default_rng(0)
        self.embed(0.01 * rng.standard_normal(self.sample_rate * 2).astype("float32"))


# --------------------------------------------------------------------------
# Silero VAD
# Streaming Silero VAD over onnxruntime (supports the v4 h/c and v5 state graphs).
# --------------------------------------------------------------------------

FRAME_SAMPLES = 512  # 32 ms at 16 kHz; fixed by the model
_V5_CONTEXT = 64


class SileroVAD:
    """Stateful Silero VAD. Call with consecutive 512-sample float32 frames."""

    def __init__(self, model_path: str, sample_rate: int = 16000) -> None:
        import onnxruntime as ort

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        opts.inter_op_num_threads = 1
        opts.log_severity_level = 3
        self.session = ort.InferenceSession(
            model_path, sess_options=opts, providers=["CPUExecutionProvider"]
        )
        names = {i.name for i in self.session.get_inputs()}
        self.is_v5 = "state" in names
        self.sample_rate = sample_rate
        self._sr = np.array(sample_rate, dtype=np.int64)
        self.reset()

    def reset(self) -> None:
        if self.is_v5:
            self._state = np.zeros((2, 1, 128), dtype=np.float32)
            self._context = np.zeros(_V5_CONTEXT, dtype=np.float32)
        else:
            self._h = np.zeros((2, 1, 64), dtype=np.float32)
            self._c = np.zeros((2, 1, 64), dtype=np.float32)

    def __call__(self, frame: np.ndarray) -> float:
        frame = np.asarray(frame, dtype=np.float32)
        if frame.shape[0] != FRAME_SAMPLES:
            raise ValueError(f"Silero expects {FRAME_SAMPLES} samples per frame")
        if self.is_v5:
            x = np.concatenate([self._context, frame])[None, :]
            out, self._state = self.session.run(
                None, {"input": x, "state": self._state, "sr": self._sr}
            )
            self._context = x[0, -_V5_CONTEXT:]
        else:
            out, self._h, self._c = self.session.run(
                None,
                {"input": frame[None, :], "h": self._h, "c": self._c, "sr": self._sr},
            )
        return float(np.asarray(out).reshape(-1)[0])


# --------------------------------------------------------------------------
# Segmenter
# Speech segmentation on top of Silero probabilities.
#
# Per 32 ms frame:
#
# * hysteresis on the Silero probability (``vad_on`` to open, ``vad_off`` to keep),
# * adaptive energy gate: to *open* a segment the frame must be ``energy_margin_db``
#   above a rolling noise floor (EMA of non-speech frames),
# * ``min_speech_sec`` of consecutive speech to open, ``min_silence_sec`` to close,
# * ``pad_sec`` of audio kept before the onset and after the offset,
# * ``mute()`` (robot TTS) closes the current segment and suppresses new ones.
# --------------------------------------------------------------------------


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


# --------------------------------------------------------------------------
# Profiles
# Enrolled speaker profiles persisted as ``<dir>/<name>.npz``.
#
# A profile keeps a fixed ``anchor`` (mean of the enrollment embeddings, never
# modified afterwards) plus a small FIFO ``pool`` of high-confidence embeddings
# collected while the robot runs, so the profile can adapt to distance / room
# without drifting towards an impostor.
# --------------------------------------------------------------------------


def _safe_name(name: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9_-]+", "_", name.strip().lower())
    if not clean:
        raise ValueError("empty speaker name")
    return clean


@dataclass
class SpeakerProfile:
    name: str
    anchor: np.ndarray
    enroll_embs: np.ndarray  # [N, D]
    backend: str
    enrolled_sec: float
    consistency: float
    pool: list[np.ndarray] = field(default_factory=list)
    created: float = field(default_factory=time.time)

    def score(self, emb: np.ndarray, pool_topk: int = 3) -> float:
        """Cosine score; the adaptive pool can raise it but weighs at most 50%."""
        base = float(np.dot(self.anchor, emb))
        if len(self.pool) >= 3:
            pool_scores = np.sort(np.stack(self.pool) @ emb)[::-1][:pool_topk]
            return 0.5 * base + 0.5 * max(base, float(np.mean(pool_scores)))
        return base

    def add_to_pool(self, emb: np.ndarray, pool_max: int) -> None:
        self.pool.append(l2_normalize(emb))
        if len(self.pool) > pool_max:
            self.pool.pop(0)

    def summary(self) -> dict:
        return {
            "name": self.name,
            "backend": self.backend,
            "enrolled_sec": round(self.enrolled_sec, 2),
            "consistency": round(self.consistency, 3),
            "n_enroll": int(self.enroll_embs.shape[0]),
            "pool": len(self.pool),
            "created": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.created)),
        }


class ProfileStore:
    def __init__(self, directory: str, backend: str) -> None:
        self.directory = directory
        self.backend = backend
        self.profiles: dict[str, SpeakerProfile] = {}
        os.makedirs(directory, exist_ok=True)

    def path(self, name: str) -> str:
        return os.path.join(self.directory, f"{_safe_name(name)}.npz")

    def load_all(self) -> list[str]:
        """Load every profile of the current backend. Returns skipped files."""
        self.profiles.clear()
        skipped = []
        for fname in sorted(os.listdir(self.directory)):
            if not fname.endswith(".npz"):
                continue
            try:
                prof = self._load(os.path.join(self.directory, fname))
            except Exception as exc:  # corrupted file
                skipped.append(f"{fname}: {exc}")
                continue
            if prof.backend != self.backend:
                skipped.append(f"{fname}: backend {prof.backend} != {self.backend}")
                continue
            self.profiles[prof.name] = prof
        return skipped

    def _load(self, path: str) -> SpeakerProfile:
        data = np.load(path, allow_pickle=False)
        meta = json.loads(str(data["meta"]))
        pool = list(data["pool"]) if data["pool"].size else []
        return SpeakerProfile(
            name=meta["name"],
            anchor=data["anchor"].astype(np.float32),
            enroll_embs=data["enroll_embs"].astype(np.float32),
            backend=meta["backend"],
            enrolled_sec=float(meta["enrolled_sec"]),
            consistency=float(meta["consistency"]),
            pool=[p.astype(np.float32) for p in pool],
            created=float(meta.get("created", time.time())),
        )

    def save(self, prof: SpeakerProfile) -> str:
        meta = {
            "name": prof.name,
            "backend": prof.backend,
            "enrolled_sec": prof.enrolled_sec,
            "consistency": prof.consistency,
            "created": prof.created,
        }
        pool = (
            np.stack(prof.pool)
            if prof.pool
            else np.zeros((0, prof.anchor.shape[0]), np.float32)
        )
        path = self.path(prof.name)
        tmp = path + ".tmp.npz"
        np.savez(
            tmp,
            anchor=prof.anchor,
            enroll_embs=prof.enroll_embs,
            pool=pool,
            meta=np.array(json.dumps(meta)),
        )
        os.replace(tmp, path)
        self.profiles[prof.name] = prof
        return path

    def forget(self, name: str) -> bool:
        key = _safe_name(name)
        existed = False
        for pname in list(self.profiles):
            if _safe_name(pname) == key:
                del self.profiles[pname]
                existed = True
        path = self.path(name)
        if os.path.exists(path):
            os.remove(path)
            existed = True
        return existed

    def get(self, name: str) -> SpeakerProfile | None:
        key = _safe_name(name)
        for pname, prof in self.profiles.items():
            if _safe_name(pname) == key:
                return prof
        return None


# --------------------------------------------------------------------------
# Verifier
# Scoring, decisions and enrollment for speaker verification.
# --------------------------------------------------------------------------

TARGET = "TARGET"
NOT_TARGET = "NOT_TARGET"
UNKNOWN = "UNKNOWN"
MIXED = "MIXED"  # target and someone else both spoke in the same segment
NO_PROFILES = "NO_PROFILES"


@dataclass
class VerifierConfig:
    accept_th: float = 0.38
    reject_th: float = 0.24
    margin: float = 0.10
    min_sec: float = 1.0  # shorter audio -> UNKNOWN
    adapt_th: float = 0.65
    adapt_min_sec: float = 2.0
    pool_max: int = 10
    enroll_min_sec: float = 3.0
    enroll_good_sec: float = 6.0
    enroll_win_sec: float = 2.0
    enroll_hop_sec: float = 1.0
    enroll_outlier_th: float = 0.45
    enroll_min_consistency: float = 0.55


@dataclass
class Decision:
    label: str
    target: str
    best: str
    score: float
    second: str
    second_score: float
    target_score: float
    scores: dict[str, float]

    def as_dict(self) -> dict:
        return {
            "label": self.label,
            "target": self.target,
            "best": self.best,
            "score": round(self.score, 4),
            "second": self.second,
            "second_score": round(self.second_score, 4),
            "target_score": round(self.target_score, 4),
            "scores": {k: round(v, 4) for k, v in self.scores.items()},
        }


def decide(
    emb: np.ndarray,
    profiles: dict[str, SpeakerProfile],
    target: str,
    duration: float,
    cfg: VerifierConfig,
) -> Decision:
    """Decide whether ``emb`` belongs to ``target`` (or to any enrolled profile
    when ``target`` is empty)."""
    if not profiles:
        return Decision(NO_PROFILES, target, "", 0.0, "", 0.0, 0.0, {})

    scores = {name: prof.score(emb) for name, prof in profiles.items()}
    ranked = sorted(scores.items(), key=lambda kv: kv[1], reverse=True)
    best, best_score = ranked[0]
    second, second_score = ranked[1] if len(ranked) > 1 else ("", -1.0)

    tgt = target if target in scores else best
    tgt_score = scores[tgt]
    others = [s for n, s in scores.items() if n != tgt]
    best_other = max(others) if others else -1.0

    if duration < cfg.min_sec:
        label = UNKNOWN
    elif tgt_score >= cfg.accept_th and tgt_score - best_other >= cfg.margin:
        label = TARGET
    elif tgt_score <= cfg.reject_th or best_other - tgt_score >= cfg.margin:
        label = NOT_TARGET
    else:
        label = UNKNOWN

    return Decision(
        label,
        tgt if target else "",
        best,
        best_score,
        second,
        second_score,
        tgt_score,
        scores,
    )


def aggregate_segment(
    whole_label: str, window_labels: list[str], min_windows: int = 2
) -> str:
    """Final label of a segment from its whole-segment decision and the labels of
    its sliding windows (partials).

    In noisy rooms one segment often contains several voices; the whole-segment
    embedding then mixes them. Windows show whether the target spoke at all:

    * target windows and other-speaker windows both present -> MIXED
    * mostly target windows -> TARGET (even if the whole embedding was unsure)
    * otherwise the whole-segment decision stands.
    """
    n_tgt = sum(1 for lbl in window_labels if lbl == TARGET)
    n_not = sum(1 for lbl in window_labels if lbl == NOT_TARGET)
    if n_tgt >= min_windows and n_not >= min_windows:
        return MIXED
    if whole_label == TARGET:
        return TARGET
    if n_tgt >= min_windows and n_tgt >= 0.5 * len(window_labels):
        return TARGET
    return whole_label


def windows(wave: np.ndarray, sr: int, win_sec: float, hop_sec: float) -> list:
    """Split into windows; audio shorter than 1.5 windows is returned whole."""
    win, hop = int(win_sec * sr), int(hop_sec * sr)
    if wave.shape[0] < int(1.5 * win):
        return [wave]
    return [wave[s : s + win] for s in range(0, wave.shape[0] - win + 1, hop)]


@dataclass
class EnrollResult:
    ok: bool
    quality: str  # "strong" | "weak" | "insufficient"
    message: str
    profile: SpeakerProfile | None
    kept_sec: float
    consistency: float
    n_windows: int
    n_kept: int


def build_profile(
    name: str,
    waves: list[np.ndarray],
    embedder: SpeakerEmbedder,
    backend: str,
    cfg: VerifierConfig,
) -> EnrollResult:
    """Create a profile from enrollment speech segments (float32, 16 kHz).

    Windows that disagree with the median embedding (e.g. a second voice) are
    dropped before averaging.
    """
    sr = embedder.sample_rate
    embs, lens = [], []
    for wave in waves:
        if wave.shape[0] < int(0.8 * sr):
            continue
        for w in windows(wave, sr, cfg.enroll_win_sec, cfg.enroll_hop_sec):
            embs.append(embedder.embed(w))
            lens.append(w.shape[0] / sr)
    total_sec = sum(w.shape[0] for w in waves) / sr
    if not embs:
        return EnrollResult(
            False,
            "insufficient",
            f"no usable speech (got {total_sec:.1f}s)",
            None,
            0.0,
            0.0,
            0,
            0,
        )

    mat = np.stack(embs)
    median = l2_normalize(np.median(mat, axis=0))
    keep = (mat @ median) >= cfg.enroll_outlier_th
    if keep.sum() == 0:
        keep[:] = True
    kept = mat[keep]
    anchor = l2_normalize(kept.mean(axis=0))
    consistency = float(np.mean(kept @ anchor))
    # windows overlap, so measure kept speech on the source segments instead
    kept_sec = min(total_sec, float(np.sum(np.array(lens)[keep])))

    if kept_sec < cfg.enroll_min_sec:
        return EnrollResult(
            False,
            "insufficient",
            f"only {kept_sec:.1f}s of consistent speech, need {cfg.enroll_min_sec:.1f}s",
            None,
            kept_sec,
            consistency,
            len(embs),
            int(keep.sum()),
        )
    strong = (
        kept_sec >= cfg.enroll_good_sec and consistency >= cfg.enroll_min_consistency
    )
    prof = SpeakerProfile(
        name=name,
        anchor=anchor,
        enroll_embs=kept,
        backend=backend,
        enrolled_sec=kept_sec,
        consistency=consistency,
    )
    return EnrollResult(
        True,
        "strong" if strong else "weak",
        f"enrolled {name}: {kept_sec:.1f}s, consistency {consistency:.2f}, "
        f"{int(keep.sum())}/{len(embs)} windows kept",
        prof,
        kept_sec,
        consistency,
        len(embs),
        int(keep.sum()),
    )
