"""Benchmark the trained openWakeWord models against the per-word test features.

For each wake word we load the features that train.py produced under
``.data/models/<word>/{positive,negative}_features_test.npy`` and evaluate
both the old model in ``assets/oww/`` and the new one in ``assets/oww_rt/``
over a sweep of thresholds. For each model we also report the best
threshold-accuracy pair and the operating point at the default 0.5.

Evaluating every model against the same ``frida`` test set (the previous
behavior) is wrong: a good ``yes`` model should score 0% on frida clips
because it must *not* fire on them. That's why the old benchmark showed
0% recall on yes/no/stop — the test set was specific to frida.
"""

import os

import numpy as np
import onnxruntime as ort
from tqdm import tqdm

WORDS = ["frida", "yes", "no", "stop"]
THRESHOLDS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]

REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)
FEATURES_ROOT = os.path.join(
    REPO_ROOT, "hri", "packages", "speech", "training", "oww", ".data", "models"
)
OLD_MODELS_DIR = os.path.join(REPO_ROOT, "hri", "packages", "speech", "assets", "oww")
NEW_MODELS_DIR = os.path.join(
    REPO_ROOT, "hri", "packages", "speech", "assets", "oww_rt"
)


def score_model(model_path, pos_features, neg_features):
    session = ort.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name

    pos_preds = np.zeros(len(pos_features), dtype=np.float32)
    for i, x in enumerate(tqdm(pos_features, desc="  positives", leave=False)):
        batch = x[np.newaxis, ...].astype(np.float32)
        pos_preds[i] = session.run(None, {input_name: batch})[0].item()

    neg_preds = np.zeros(len(neg_features), dtype=np.float32)
    for i, x in enumerate(tqdm(neg_features, desc="  negatives", leave=False)):
        batch = x[np.newaxis, ...].astype(np.float32)
        neg_preds[i] = session.run(None, {input_name: batch})[0].item()

    return pos_preds, neg_preds


def metrics_at_threshold(pos_preds, neg_preds, threshold):
    tp = int(np.sum(pos_preds >= threshold))
    fn = int(np.sum(pos_preds < threshold))
    tn = int(np.sum(neg_preds < threshold))
    fp = int(np.sum(neg_preds >= threshold))
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    accuracy = (tp + tn) / (len(pos_preds) + len(neg_preds))
    # Approximate F1 to give a single-number ranking per threshold.
    f1 = (
        2 * precision * recall / (precision + recall)
        if (precision + recall) > 0
        else 0.0
    )
    return {
        "threshold": threshold,
        "recall": recall,
        "precision": precision,
        "accuracy": accuracy,
        "fp": fp,
        "tp": tp,
        "f1": f1,
    }


def format_row(m):
    return (
        f"  thr={m['threshold']:.2f}  "
        f"rec={m['recall']:.3f}  "
        f"prec={m['precision']:.3f}  "
        f"acc={m['accuracy']:.3f}  "
        f"tp={m['tp']}  fp={m['fp']}  "
        f"f1={m['f1']:.3f}"
    )


def evaluate(word, model_path, pos_features, neg_features):
    print(f"\n[{word}] {model_path}")
    if not os.path.exists(model_path):
        print("  (model not found)")
        return
    pos_preds, neg_preds = score_model(model_path, pos_features, neg_features)
    rows = [metrics_at_threshold(pos_preds, neg_preds, t) for t in THRESHOLDS]
    for r in rows:
        print(format_row(r))
    best = max(rows, key=lambda r: r["f1"])
    default = next(r for r in rows if abs(r["threshold"] - 0.5) < 1e-6)
    print(
        f"  BEST F1 -> thr={best['threshold']:.2f} "
        f"(rec={best['recall']:.3f}, prec={best['precision']:.3f}, f1={best['f1']:.3f})"
    )
    print(
        f"  @0.5    -> rec={default['recall']:.3f}, "
        f"prec={default['precision']:.3f}, fp={default['fp']}"
    )


def main():
    for word in WORDS:
        pos_path = os.path.join(FEATURES_ROOT, word, "positive_features_test.npy")
        neg_path = os.path.join(FEATURES_ROOT, word, "negative_features_test.npy")
        if not (os.path.exists(pos_path) and os.path.exists(neg_path)):
            print(
                f"\n[{word}] skipping — test features not found at "
                f"{os.path.dirname(pos_path)}"
            )
            continue
        pos = np.load(pos_path)
        neg = np.load(neg_path)
        print(
            f"\n==== {word}  ({pos.shape[0]} positives, {neg.shape[0]} negatives) ===="
        )
        evaluate(word, os.path.join(OLD_MODELS_DIR, f"{word}.onnx"), pos, neg)
        evaluate(word, os.path.join(NEW_MODELS_DIR, f"{word}.onnx"), pos, neg)


if __name__ == "__main__":
    main()
