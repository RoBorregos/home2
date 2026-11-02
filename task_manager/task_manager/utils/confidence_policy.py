#!/usr/bin/env python3

"""
Confidence-calibrated clarification policy for HRITasks.ask_and_confirm (issue #1384).

Pure python (no rclpy) so it can be unit tested and reused by the offline analysis scripts in
research/confidence_calibration/.

Decisions:
    ACCEPT  -> return the extracted value without asking.
    REPEAT  -> "Sorry, I couldn't hear you, could you repeat?" (not a non-essential question in HRIC).
    CONFIRM -> "Did you say X?" (non-essential question in HRIC).
"""

import difflib
import json
import math
import os
import re
from datetime import datetime
from pathlib import Path
from typing import Optional

ACCEPT = "accept"
REPEAT = "repeat"
CONFIRM = "confirm"

POLICIES = ("legacy", "binary", "three_way")

# Same layout as the display evidence folder (display_ui.evidence_root)
_WORKSPACE_SRC = Path("/workspace/src")


def _normalize(text: str) -> str:
    return re.sub(r"[^\w\s]", "", text or "").strip().lower()


def decisions_root() -> Path:
    if _WORKSPACE_SRC.is_dir():
        return _WORKSPACE_SRC / "logs" / "hri_decisions"
    return Path.home() / ".frida" / "hri_decisions"


def match_confidences(
    target: str, words: list[str], confidences: list[float], fuzzy_cutoff: float = 0.75
) -> tuple[list[float], str]:
    """
    Find the STT confidences of the words that make up the extracted value.

    Returns (matched_confidences, match_type) where match_type is:
        exact -> every target word appears in the transcription
        fuzzy -> the closest span of the transcription (e.g. "coke" for "Coca-Cola")
        whole -> nothing matched, fall back to the whole transcription
        none  -> there is no transcription confidence at all
    """
    norm_words = [_normalize(w) for w in words]
    pairs = [(w, c) for w, c in zip(norm_words, confidences) if w]
    if not pairs:
        return [], "none"

    target_words = _normalize(target).split()
    lookup = {}
    for w, c in pairs:
        # Keep the lowest confidence when a word is repeated
        lookup[w] = min(c, lookup.get(w, c))

    if target_words and all(w in lookup for w in target_words):
        return [lookup[w] for w in target_words], "exact"

    # Partial or normalized value: best contiguous span of the transcription
    target_str = " ".join(target_words)
    best_score, best_span = 0.0, None
    if target_str:
        max_len = min(len(pairs), len(target_words) + 2)
        for size in range(1, max_len + 1):
            for start in range(len(pairs) - size + 1):
                span = pairs[start : start + size]
                score = difflib.SequenceMatcher(
                    None, target_str, " ".join(w for w, _ in span)
                ).ratio()
                if score > best_score:
                    best_score, best_span = score, span
    if best_span is not None and best_score >= fuzzy_cutoff:
        return [c for _, c in best_span], "fuzzy"

    return [c for _, c in pairs], "whole"


def legacy_matched_confidences(target: str, word_confidences: dict) -> list[float]:
    """Matching used before #1384 (kept for the legacy policy and for comparisons)."""
    normalized = {_normalize(w): c for w, c in word_confidences.items()}
    return [normalized[w] for w in _normalize(target).split() if w in normalized]


def compute_features(
    target: str,
    transcription: str,
    words: list[str],
    confidences: list[float],
    similarity: float = 1.0,
    exact_match: bool = False,
    options_used: bool = False,
    stt_meta: Optional[dict] = None,
) -> dict:
    matched, match_type = match_confidences(target, words, confidences)
    whole = [c for w, c in zip(words, confidences) if _normalize(w)]
    legacy = legacy_matched_confidences(target, dict(zip(words, confidences)))

    def mean(xs):
        return sum(xs) / len(xs) if xs else 0.0

    features = {
        "match_type": match_type,
        "matched_confidences": matched,
        "conf_mean": mean(matched),
        "conf_min": min(matched) if matched else 0.0,
        "conf_prod": math.prod(matched) if matched else 0.0,
        "conf_whole_mean": mean(whole),
        "conf_whole_min": min(whole) if whole else 0.0,
        "legacy_conf": mean(legacy),
        "similarity": float(similarity),
        "exact_match": bool(exact_match),
        "options_used": bool(options_used),
        "normalized_flag": _normalize(target) not in _normalize(transcription),
        "n_words": len(whole),
        "n_target_words": len(_normalize(target).split()),
    }
    for key in ("avg_logprob", "no_speech_prob"):
        if stt_meta and key in stt_meta:
            features[key] = float(stt_meta[key])
    return features


class CalibratedModel:
    """
    p(correct | features) exported by research/confidence_calibration/analyze.py.

    JSON format:
        {"type": "isotonic", "feature": "conf_min", "x": [...], "y": [...]}
        {"type": "logistic", "features": [...], "coef": [...], "intercept": b,
         "mean": [...], "scale": [...], "post_isotonic": {"x": [...], "y": [...]} (optional)}
    """

    def __init__(self, spec: dict):
        self.spec = spec

    @classmethod
    def load(cls, path: str) -> Optional["CalibratedModel"]:
        if not path or not os.path.isfile(path):
            return None
        with open(path) as f:
            return cls(json.load(f))

    @staticmethod
    def feature_vector(features: dict, names: list[str]) -> list[float]:
        vec = []
        for name in names:
            value = features.get(name, 0.0)
            vec.append(float(value) if not isinstance(value, str) else 0.0)
        return vec

    @staticmethod
    def interpolate(x: float, xs: list[float], ys: list[float]) -> float:
        if x <= xs[0]:
            return ys[0]
        if x >= xs[-1]:
            return ys[-1]
        for i in range(1, len(xs)):
            if x <= xs[i]:
                t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]) if xs[i] > xs[i - 1] else 0
                return ys[i - 1] + t * (ys[i] - ys[i - 1])
        return ys[-1]

    def predict(self, features: dict) -> float:
        spec = self.spec
        if spec["type"] == "isotonic":
            return self.interpolate(float(features.get(spec["feature"], 0.0)), spec["x"], spec["y"])
        if spec["type"] == "logistic":
            vec = self.feature_vector(features, spec["features"])
            z = spec["intercept"]
            for v, c, m, s in zip(vec, spec["coef"], spec["mean"], spec["scale"]):
                z += c * (v - m) / (s or 1.0)
            p = 1.0 / (1.0 + math.exp(-z))
            post = spec.get("post_isotonic")
            return self.interpolate(p, post["x"], post["y"]) if post else p
        raise ValueError(f"Unknown model type: {spec['type']}")


def decide(
    features: dict,
    policy: str = "legacy",
    thresholds: Optional[dict] = None,
    model: Optional[CalibratedModel] = None,
    legacy_similarity: float = 0.7,
    legacy_confidence: float = 0.7,
    fix_matching: bool = True,
) -> tuple[str, Optional[float]]:
    """
    Returns (decision, p_correct). p_correct is None for the legacy policy.

    legacy: same rule as before #1384 (exact option match or similarity/avg confidence > 0.7).
        With fix_matching=False it also uses the old word matching (0 confidence for normalized values).
    binary: accept if p >= accept, else confirm.
    three_way: accept if p >= accept, repeat if p < repeat, else confirm.
    """
    if policy not in POLICIES:
        raise ValueError(f"Unknown policy: {policy}")

    if policy == "legacy" or model is None:
        if features["exact_match"]:
            return ACCEPT, None
        confidence = features["conf_mean"] if fix_matching else features["legacy_conf"]
        if features["similarity"] > legacy_similarity and confidence > legacy_confidence:
            return ACCEPT, None
        return CONFIRM, None

    thresholds = thresholds or {}
    p = model.predict(features)
    if p >= thresholds.get("accept", 0.9):
        return ACCEPT, p
    if policy == "three_way" and p < thresholds.get("repeat", 0.3):
        return REPEAT, p
    return CONFIRM, p


class DecisionLogger:
    """One JSONL record per ask_and_confirm attempt."""

    def __init__(self, task: str = "default", root: Optional[Path] = None, enabled: bool = True):
        self.enabled = enabled
        self.path = None
        if not enabled:
            return
        root = Path(root) if root else decisions_root()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = root / f"{task}_{stamp}.jsonl"
        self.session = {
            "speaker_id": os.environ.get("FRIDA_SPEAKER_ID", ""),
            "condition": os.environ.get("FRIDA_CONDITION", ""),
        }

    def log(self, record: dict) -> None:
        if not self.enabled:
            return
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            entry = {"timestamp": datetime.now().isoformat(), **self.session, **record}
            with open(self.path, "a") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except OSError as e:
            print("Failed writing decision log:", e)
