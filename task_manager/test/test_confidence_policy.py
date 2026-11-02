#!/usr/bin/env python3

"""Unit tests for the ask_and_confirm confidence policy (#1384). No ROS needed:

python -m pytest task_manager/test/test_confidence_policy.py
"""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "task_manager", "utils"))

import confidence_policy as cp  # noqa: E402


def features(target, transcription, confidences=None, **kwargs):
    words = transcription.split()
    confidences = confidences if confidences is not None else [0.9] * len(words)
    return cp.compute_features(target, transcription, words, confidences, **kwargs)


def old_rule(target, word_confidences, similarity=1.0, exact_match=False):
    """Decision of ask_and_confirm before #1384 (hri_tasks.py, 0.7/0.7 thresholds)."""
    if exact_match:
        return cp.ACCEPT
    matched = cp.legacy_matched_confidences(target, word_confidences)
    avg = sum(matched) / len(matched) if matched else 0
    return cp.ACCEPT if similarity > 0.7 and avg > 0.7 else cp.CONFIRM


# --- Normalization bug ---


def test_normalized_value_no_longer_has_zero_confidence():
    f = features("Coca-Cola", "I would like a coke please", [0.9, 0.9, 0.95, 0.9, 0.92, 0.9])
    assert f["legacy_conf"] == 0  # the old bug
    assert f["conf_mean"] > 0.85
    assert f["normalized_flag"]
    assert cp.decide(f, "legacy")[0] == cp.ACCEPT
    assert cp.decide(f, "legacy", fix_matching=False)[0] == cp.CONFIRM


def test_fuzzy_match_uses_closest_span():
    matched, match_type = cp.match_confidences(
        "Coca Cola", ["I", "want", "coca-cola", "now"], [0.99, 0.99, 0.4, 0.99]
    )
    assert match_type == "fuzzy"
    assert matched == [0.4]


def test_whole_transcription_fallback():
    matched, match_type = cp.match_confidences(
        "Mohammed", ["my", "name", "is", "Joe"], [0.9, 0.8, 0.7, 0.6]
    )
    assert match_type == "whole"
    assert matched == [0.9, 0.8, 0.7, 0.6]


def test_exact_match_and_punctuation():
    matched, match_type = cp.match_confidences(
        "John Smith", ["I'm", "John,", "Smith."], [0.5, 0.8, 0.6]
    )
    assert match_type == "exact"
    assert matched == [0.8, 0.6]


def test_empty_transcription():
    f = features("John", "", [])
    assert f["match_type"] == "none"
    assert f["conf_mean"] == 0
    assert cp.decide(f, "legacy")[0] == cp.CONFIRM


def test_min_and_product_are_conservative():
    f = features("Anna Maria", "Anna Maria", [0.95, 0.3])
    assert f["conf_min"] == 0.3
    assert f["conf_prod"] == pytest.approx(0.285)
    assert f["conf_mean"] == pytest.approx(0.625)


# --- Legacy parity: with fix_matching=False the decision is exactly the old one ---

PARITY_CASES = [
    ("John", "my name is John", [0.9, 0.9, 0.9, 0.95], 1.0, False),
    ("John", "my name is John", [0.9, 0.9, 0.9, 0.5], 1.0, False),
    ("Coca-Cola", "coke", [0.95], 1.0, False),
    ("water", "water please", [0.8, 0.9], 0.6, False),
    ("water", "water please", [0.8, 0.9], 0.8, False),
    ("kitchen", "the kitchen", [0.2, 0.3], 1.0, True),
    ("Anna Maria", "Anna Maria", [0.95, 0.5], 1.0, False),
]


@pytest.mark.parametrize("target,transcription,confs,similarity,exact", PARITY_CASES)
def test_legacy_parity(target, transcription, confs, similarity, exact):
    f = features(target, transcription, confs, similarity=similarity, exact_match=exact)
    expected = old_rule(target, dict(zip(transcription.split(), confs)), similarity, exact)
    assert cp.decide(f, "legacy", fix_matching=False)[0] == expected


def test_legacy_fixed_only_differs_on_unmatched_values():
    for target, transcription, confs, similarity, exact in PARITY_CASES:
        f = features(target, transcription, confs, similarity=similarity, exact_match=exact)
        if f["match_type"] == "exact":
            assert cp.decide(f, "legacy")[0] == cp.decide(f, "legacy", fix_matching=False)[0]


# --- Calibrated policies ---

ISOTONIC = cp.CalibratedModel(
    {"type": "isotonic", "feature": "conf_min", "x": [0.0, 0.5, 1.0], "y": [0.1, 0.5, 1.0]}
)


def test_isotonic_interpolation():
    assert ISOTONIC.predict({"conf_min": 0.25}) == pytest.approx(0.3)
    assert ISOTONIC.predict({"conf_min": -1}) == 0.1
    assert ISOTONIC.predict({"conf_min": 2}) == 1.0


def test_logistic_model():
    model = cp.CalibratedModel(
        {
            "type": "logistic",
            "features": ["conf_min", "exact_match"],
            "coef": [2.0, 1.0],
            "intercept": 0.0,
            "mean": [0.5, 0.0],
            "scale": [1.0, 1.0],
        }
    )
    assert model.predict({"conf_min": 0.5, "exact_match": False}) == pytest.approx(0.5)
    assert model.predict({"conf_min": 1.0, "exact_match": True}) > 0.85


@pytest.mark.parametrize(
    "conf,binary,three_way",
    [
        (0.99, cp.ACCEPT, cp.ACCEPT),
        (0.6, cp.CONFIRM, cp.CONFIRM),
        (0.1, cp.CONFIRM, cp.REPEAT),
    ],
)
def test_thresholds(conf, binary, three_way):
    f = features("John", "John", [conf])
    thresholds = {"accept": 0.9, "repeat": 0.3}
    assert cp.decide(f, "binary", thresholds, ISOTONIC)[0] == binary
    assert cp.decide(f, "three_way", thresholds, ISOTONIC)[0] == three_way


def test_calibrated_without_model_falls_back_to_legacy():
    f = features("John", "John", [0.1])
    assert cp.decide(f, "three_way", {"accept": 0.9, "repeat": 0.3}, None) == (cp.CONFIRM, None)


def test_unknown_policy():
    with pytest.raises(ValueError):
        cp.decide(features("John", "John"), "always")


# --- Decision log ---


def test_decision_logger(tmp_path, monkeypatch):
    monkeypatch.setenv("FRIDA_SPEAKER_ID", "s01")
    logger = cp.DecisionLogger("hric", root=tmp_path)
    logger.log({"decision": cp.ACCEPT, "features": features("John", "John")})
    logger.log({"decision": cp.CONFIRM})
    lines = logger.path.read_text().splitlines()
    assert len(lines) == 2
    first = json.loads(lines[0])
    assert first["speaker_id"] == "s01"
    assert first["features"]["match_type"] == "exact"


def test_model_load(tmp_path):
    path = tmp_path / "model.json"
    path.write_text(json.dumps(ISOTONIC.spec))
    assert cp.CalibratedModel.load(str(path)).predict({"conf_min": 0.5}) == 0.5
    assert cp.CalibratedModel.load(str(tmp_path / "missing.json")) is None
