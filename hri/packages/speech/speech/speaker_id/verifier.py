"""Scoring, decisions and enrollment for speaker verification."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from speech.speaker_id.embedder import SpeakerEmbedder, l2_normalize
from speech.speaker_id.profiles import ProfileStore, SpeakerProfile

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


__all__ = [
    "TARGET",
    "NOT_TARGET",
    "UNKNOWN",
    "MIXED",
    "NO_PROFILES",
    "VerifierConfig",
    "Decision",
    "decide",
    "aggregate_segment",
    "build_profile",
    "EnrollResult",
    "ProfileStore",
]
