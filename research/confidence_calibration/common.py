"""
Shared helpers for the ask_and_confirm calibration study (#1384).

Reads the decision logs written by HRITasks.ask_and_confirm (logs/hri_decisions/*.jsonl), joins
them with the human labels (labels.jsonl) and builds feature matrices. The feature definitions come
from task_manager/utils/confidence_policy.py so offline and on-robot features are identical.
"""

import glob
import json
import os
import random
import re
import sys
from collections import defaultdict
from difflib import SequenceMatcher

import jellyfish
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "task_manager", "task_manager", "utils"))

import confidence_policy as cp  # noqa: E402

DATA_DIR = os.path.join(HERE, "data")
PAPER_DIR = os.path.join(HERE, "paper")
FIG_DIR = os.path.join(PAPER_DIR, "figures")
GEN_DIR = os.path.join(PAPER_DIR, "generated")
MODEL_DIR = os.path.join(HERE, "models")

# Numeric features available on the robot (see confidence_policy.compute_features)
FEATURES = [
    "conf_mean",
    "conf_min",
    "conf_prod",
    "conf_whole_mean",
    "conf_whole_min",
    "similarity",
    "exact_match",
    "normalized_flag",
    "n_words",
    "n_target_words",
]
STT_FEATURES = ["avg_logprob", "no_speech_prob"]

# Drink aliases accepted as the same value when auto-labeling
ALIASES = {
    "coke": "cocacola",
    "cocacola": "cocacola",
    "sidral": "sidralmundet",
    "oj": "orangejuice",
    "icetea": "icedtea",
    "redbull": "redbull",
}

DECIDED = (cp.ACCEPT, cp.CONFIRM, cp.REPEAT)


def canon(text):
    key = re.sub(r"[^a-z0-9]", "", (text or "").lower())
    return ALIASES.get(key, key)


def same_value(extracted, intended, slot=""):
    a, b = canon(extracted), canon(intended)
    if not a or not b:
        return False
    if slot == "drink":
        return a == b or (len(a) > 4 and SequenceMatcher(None, a, b).ratio() >= 0.9)
    if slot == "name":
        # The robot says the name aloud, so homophone spellings (Adel / Adele) are right
        return a == b or jellyfish.metaphone(a) == jellyfish.metaphone(b)
    return a == b


def load_logs(data_dir=DATA_DIR):
    """All decision records of data_dir/logs/*.jsonl (copied from logs/hri_decisions/)."""
    paths = sorted(glob.glob(os.path.join(data_dir, "logs", "*.jsonl")))
    records = []
    for path in paths:
        with open(path, encoding="utf-8") as f:
            for i, line in enumerate(f):
                if not line.strip():
                    continue
                rec = json.loads(line)
                rec["id"] = f"{os.path.basename(path)}:{i}"
                rec["source"] = os.path.basename(path)
                records.append(rec)
    return records


def load_labels(data_dir=DATA_DIR):
    path = os.path.join(data_dir, "labels.jsonl")
    labels = {}
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            for line in f:
                if line.strip():
                    entry = json.loads(line)
                    labels[entry["id"]] = entry  # last one wins
    return labels


def auto_label(rec):
    """(correct, source) from the intended value or the confirmation answer, or (None, None)."""
    if not rec.get("extracted_value"):
        return None, None
    if rec.get("intended"):
        return same_value(
            rec["extracted_value"], rec["intended"], rec.get("slot", "")
        ), "intended"
    if rec.get("confirm_answer") in ("yes", "no"):
        return rec["confirm_answer"] == "yes", "confirm_answer"
    return None, None


def labeled_attempts(data_dir=DATA_DIR, task=None):
    """Records with a decision, an extracted value and a label (human label > intended > yes/no)."""
    records = load_logs(data_dir)
    labels = load_labels(data_dir)
    out = []
    for rec in records:
        if rec.get("decision") not in DECIDED or not rec.get("extracted_value"):
            continue
        if task and rec.get("task") != task:
            continue
        if rec["id"] in labels and labels[rec["id"]].get("correct") is not None:
            correct, source = labels[rec["id"]]["correct"], "human"
        else:
            correct, source = auto_label(rec)
        if correct is None:
            continue
        out.append({**rec, "correct": bool(correct), "label_source": source})
    return out


def feature_names(attempts):
    names = list(FEATURES)
    for key in STT_FEATURES:
        if attempts and all(key in a["features"] for a in attempts):
            names.append(key)
    return names


def matrix(attempts, names):
    X = np.array(
        [cp.CalibratedModel.feature_vector(a["features"], names) for a in attempts]
    )
    y = np.array([int(a["correct"]) for a in attempts])
    return X, y


def speaker_split(attempts, calib_ratio=0.7, seed=0):
    """Split by speaker: no speaker appears in both sets."""
    speakers = sorted({a.get("speaker_id") or a["source"] for a in attempts})
    rng = random.Random(seed)
    rng.shuffle(speakers)
    n_calib = max(1, round(len(speakers) * calib_ratio))
    calib = set(speakers[:n_calib])
    key = lambda a: a.get("speaker_id") or a["source"]  # noqa: E731
    return (
        [a for a in attempts if key(a) in calib],
        [a for a in attempts if key(a) not in calib],
        sorted(calib),
        sorted(set(speakers) - calib),
    )


def slots(attempts):
    """Group attempts into dialog slots (one ask_and_confirm call): speaker, trial, slot."""
    groups = defaultdict(list)
    for a in attempts:
        key = (
            a.get("speaker_id"),
            a.get("source"),
            a.get("trial"),
            a.get("slot") or a.get("query"),
        )
        groups[key].append(a)
    for group in groups.values():
        group.sort(key=lambda a: a.get("attempt_idx", 0))
    return list(groups.values())


def ensure_dirs():
    for d in (FIG_DIR, GEN_DIR, MODEL_DIR):
        os.makedirs(d, exist_ok=True)


def write_macros(path, macros):
    """LaTeX \\newcommand file so the paper picks up regenerated numbers."""
    with open(path, "w", encoding="utf-8") as f:
        for name, value in macros.items():
            f.write(f"\\newcommand{{\\{name}}}{{{value}}}\n")
