#!/usr/bin/env python3

"""
Week 1-3 analysis of the ask_and_confirm calibration study (#1384).

    python analyze.py --data data            # real data (data/logs + data/labels.jsonl)
    python analyze.py --data data/synthetic  # pipeline check with make_synthetic.py output

Outputs:
    paper/generated/analysis.tex      LaTeX macros + tables used by paper/main.tex
    paper/figures/*.pdf               reliability diagram, ROC, confidence histograms
    models/confidence_model.json      selected model, loadable by confidence_policy.CalibratedModel
    models/all_models.json            every fitted model
"""

import argparse
import json
import os
from collections import Counter, defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from sklearn.isotonic import IsotonicRegression  # noqa: E402
from sklearn.linear_model import LogisticRegression  # noqa: E402
from sklearn.metrics import brier_score_loss, log_loss, roc_auc_score, roc_curve  # noqa: E402
from sklearn.model_selection import GroupKFold  # noqa: E402

import common  # noqa: E402
from common import cp  # noqa: E402

# Reference categorical palette (dataviz skill), first three slots validate all-pairs
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK_2, GRID = "#0b0b0b", "#52514e", "#e4e3df"

plt.rcParams.update(
    {
        "font.size": 8,
        "axes.edgecolor": INK_2,
        "axes.labelcolor": INK,
        "xtick.color": INK_2,
        "ytick.color": INK_2,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "legend.frameon": False,
        "figure.dpi": 200,
        "savefig.bbox": "tight",
    }
)


# --- Metrics ---


def ece(y, p, bins=10):
    """Expected calibration error, equal-width bins."""
    y, p = np.asarray(y), np.clip(np.asarray(p, dtype=float), 0, 1)
    idx = np.minimum((p * bins).astype(int), bins - 1)
    total = 0.0
    for b in range(bins):
        mask = idx == b
        if mask.any():
            total += mask.mean() * abs(y[mask].mean() - p[mask].mean())
    return total


def auroc(y, p):
    return roc_auc_score(y, p) if len(set(y)) > 1 else float("nan")


def bootstrap_ci(y, p, metric, groups, n=1000, seed=0):
    """95% CI resampling speakers (cluster bootstrap)."""
    rng = np.random.default_rng(seed)
    y, p, groups = np.asarray(y), np.asarray(p), np.asarray(groups)
    uniq = np.unique(groups)
    by_group = {g: np.where(groups == g)[0] for g in uniq}
    values = []
    for _ in range(n):
        idx = np.concatenate([by_group[g] for g in rng.choice(uniq, len(uniq))])
        if len(set(y[idx])) > 1:
            values.append(metric(y[idx], p[idx]))
    return (
        np.percentile(values, [2.5, 97.5]) if values else (float("nan"), float("nan"))
    )


def metrics(y, p, groups):
    lo, hi = bootstrap_ci(y, p, auroc, groups)
    return {
        "auroc": auroc(y, p),
        "auroc_lo": lo,
        "auroc_hi": hi,
        "ece": ece(y, p),
        "brier": brier_score_loss(y, np.clip(p, 0, 1)),
        "nll": log_loss(y, np.clip(p, 1e-4, 1 - 1e-4), labels=[0, 1]),
    }


# --- Models (each returns a CalibratedModel spec) ---


def fit_isotonic(X, y, names, feature):
    x = X[:, names.index(feature)]
    iso = IsotonicRegression(out_of_bounds="clip", y_min=0.0, y_max=1.0).fit(x, y)
    return {
        "type": "isotonic",
        "feature": feature,
        "x": [float(v) for v in iso.X_thresholds_],
        "y": [float(v) for v in iso.y_thresholds_],
    }


def fit_logistic(X, y, names, features, C=1.0):
    cols = [names.index(f) for f in features]
    Xs = X[:, cols]
    mean, scale = Xs.mean(axis=0), Xs.std(axis=0)
    scale[scale == 0] = 1.0
    lr = LogisticRegression(C=C, max_iter=2000).fit((Xs - mean) / scale, y)
    return {
        "type": "logistic",
        "features": list(features),
        "coef": lr.coef_[0].tolist(),
        "intercept": float(lr.intercept_[0]),
        "mean": mean.tolist(),
        "scale": scale.tolist(),
    }


def predict(spec, attempts):
    model = cp.CalibratedModel(spec)
    return np.array([model.predict(a["features"]) for a in attempts])


def out_of_fold(fit, attempts, X, y, groups, folds=5):
    """Out-of-fold predictions grouped by speaker (for model selection and post-calibration)."""
    folds = min(folds, len(set(groups)))
    oof = np.zeros(len(y))
    for train, test in GroupKFold(n_splits=folds).split(X, y, groups):
        spec = fit(X[train], y[train])
        oof[test] = predict(spec, [attempts[i] for i in test])
    return oof


def fit_all(attempts, X, y, names, groups):
    """Candidate models fitted on the calibration split. Returns {name: (spec, oof)}."""
    candidates = {
        "iso_mean": lambda X_, y_: fit_isotonic(X_, y_, names, "conf_mean"),
        "iso_min": lambda X_, y_: fit_isotonic(X_, y_, names, "conf_min"),
        "platt_mean": lambda X_, y_: fit_logistic(X_, y_, names, ["conf_mean"]),
    }
    for C in (0.1, 1.0):
        candidates[f"logistic_all_C{C}"] = lambda X_, y_, C=C: fit_logistic(
            X_, y_, names, names, C
        )
    models = {}
    for name, fit in candidates.items():
        oof = out_of_fold(fit, attempts, X, y, groups)
        models[name] = (fit(X, y), oof)

    # Logistic + isotonic on its out-of-fold scores
    best_lr = min(
        (n for n in models if n.startswith("logistic")),
        key=lambda n: log_loss(y, np.clip(models[n][1], 1e-4, 1 - 1e-4)),
    )
    spec, oof = models[best_lr]
    iso = IsotonicRegression(out_of_bounds="clip", y_min=0.0, y_max=1.0).fit(oof, y)
    post = {
        "x": [float(v) for v in iso.X_thresholds_],
        "y": [float(v) for v in iso.y_thresholds_],
    }
    models["logistic_all_iso"] = ({**spec, "post_isotonic": post}, iso.predict(oof))
    return models


# --- Figures ---


def reliability_plot(path, curves, bins=10):
    fig, ax = plt.subplots(figsize=(3.3, 3.0))
    ax.plot(
        [0, 1], [0, 1], color=INK_2, lw=1, ls=(0, (3, 3)), label="Perfect calibration"
    )
    for (label, y, p), color, marker in zip(
        curves, (ORANGE, BLUE, AQUA), ("o", "s", "D")
    ):
        p = np.clip(p, 0, 1)
        idx = np.minimum((p * bins).astype(int), bins - 1)
        xs, ys, ns = [], [], []
        for b in range(bins):
            mask = idx == b
            if mask.sum() >= 3:
                xs.append(p[mask].mean())
                ys.append(y[mask].mean())
                ns.append(mask.sum())
        ax.plot(
            xs,
            ys,
            color=color,
            lw=2,
            marker=marker,
            ms=4,
            markeredgecolor="white",
            markeredgewidth=0.8,
            label=f"{label} (ECE {ece(y, p):.3f})",
        )
    ax.set_xlabel("Predicted p(correct)")
    ax.set_ylabel("Observed accuracy")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1.02)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), fontsize=6.5)
    fig.savefig(path)
    fig.savefig(path.replace(".pdf", ".png"))
    plt.close(fig)


def roc_plot(path, curves):
    fig, ax = plt.subplots(figsize=(3.3, 3.0))
    ax.plot([0, 1], [0, 1], color=INK_2, lw=1, ls=(0, (3, 3)))
    for (label, y, p), color in zip(curves, (ORANGE, BLUE, AQUA)):
        fpr, tpr, _ = roc_curve(y, p)
        ax.plot(fpr, tpr, color=color, lw=2, label=f"{label} (AUROC {auroc(y, p):.2f})")
    ax.set_xlabel("False positive rate")
    ax.set_ylabel("True positive rate")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), fontsize=6.5)
    fig.savefig(path)
    fig.savefig(path.replace(".pdf", ".png"))
    plt.close(fig)


def histogram_plot(path, attempts, feature, xlabel):
    fig, ax = plt.subplots(figsize=(3.3, 2.4))
    bins = np.linspace(0, 1, 21)
    for correct, color, label in (
        (True, BLUE, "Correct value"),
        (False, ORANGE, "Wrong value"),
    ):
        values = [a["features"][feature] for a in attempts if a["correct"] == correct]
        ax.hist(
            values,
            bins=bins,
            color=color,
            alpha=0.75,
            label=f"{label} (n={len(values)})",
            edgecolor="white",
            linewidth=0.8,
        )
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Attempts")
    ax.legend(loc="upper left", fontsize=6.5)
    fig.savefig(path)
    fig.savefig(path.replace(".pdf", ".png"))
    plt.close(fig)


# --- Week reports ---


def tex_num(x, digits=2):
    return (
        "--"
        if x is None or (isinstance(x, float) and np.isnan(x))
        else f"{x:.{digits}f}"
    )


def pct(x):
    return f"{100 * x:.1f}\\%"


def week1(attempts, macros):
    """Impact of the normalization bug on the legacy rule."""
    n = len(attempts)
    normalized = [a for a in attempts if a["features"]["normalized_flag"]]
    zero = [
        a
        for a in attempts
        if a["features"]["legacy_conf"] == 0 and a["features"]["n_words"] > 0
    ]
    decide = lambda a, fix: cp.decide(a["features"], "legacy", fix_matching=fix)[0]  # noqa: E731
    flipped = [a for a in attempts if decide(a, False) != decide(a, True)]
    flipped_wrong = [a for a in flipped if not a["correct"]]
    macros.update(
        {
            "WOneAttempts": n,
            "WOneNormalized": len(normalized),
            "WOneNormalizedPct": pct(len(normalized) / n),
            "WOneZeroConf": len(zero),
            "WOneZeroConfPct": pct(len(zero) / n),
            "WOneFlipped": len(flipped),
            "WOneFlippedPct": pct(len(flipped) / n),
            "WOneFlippedWrong": len(flipped_wrong),
            "WOneLegacyOldAccept": pct(
                np.mean([decide(a, False) == cp.ACCEPT for a in attempts])
            ),
            "WOneLegacyFixAccept": pct(
                np.mean([decide(a, True) == cp.ACCEPT for a in attempts])
            ),
        }
    )


def week2(attempts, calib, held, calib_sp, held_sp, macros):
    speakers = defaultdict(list)
    for a in attempts:
        speakers[a.get("speaker_id") or a["source"]].append(a)
    conditions = Counter()
    for a in attempts:
        for c in (a.get("condition") or "unknown").split(","):
            conditions[c.strip()] += 1
    errors = defaultdict(lambda: [0, 0])
    for a in attempts:
        for c in (a.get("condition") or "unknown").split(","):
            errors[c.strip()][0] += not a["correct"]
            errors[c.strip()][1] += 1
    sources = Counter(a["label_source"] for a in attempts)

    rows = []
    for cond, (wrong, total) in sorted(errors.items(), key=lambda kv: -kv[1][1]):
        rows.append(f"{cond} & {total} & {pct(wrong / total)} \\\\")
    with open(os.path.join(common.GEN_DIR, "conditions_table.tex"), "w") as f:
        f.write("\n".join(rows) + "\n")

    slot_err = defaultdict(lambda: [0, 0])
    for a in attempts:
        slot_err[a.get("slot") or a.get("query")][0] += not a["correct"]
        slot_err[a.get("slot") or a.get("query")][1] += 1

    macros.update(
        {
            "WTwoAttempts": len(attempts),
            "WTwoSpeakers": len(speakers),
            "WTwoCalibAttempts": len(calib),
            "WTwoHeldAttempts": len(held),
            "WTwoCalibSpeakers": len(calib_sp),
            "WTwoHeldSpeakers": len(held_sp),
            "WTwoErrorRate": pct(np.mean([not a["correct"] for a in attempts])),
            "WTwoNameErrorRate": pct(slot_err["name"][0] / max(1, slot_err["name"][1])),
            "WTwoDrinkErrorRate": pct(
                slot_err["drink"][0] / max(1, slot_err["drink"][1])
            ),
            "WTwoLabelSources": ", ".join(
                f"{k}: {v}" for k, v in sorted(sources.items())
            ),
            "WTwoTargetMet": "yes"
            if len(attempts) >= 200 and len(speakers) >= 10
            else "no",
        }
    )
    histogram_plot(
        os.path.join(common.FIG_DIR, "hist_conf_mean.pdf"),
        attempts,
        "conf_mean",
        "Mean word confidence of the extracted value",
    )
    histogram_plot(
        os.path.join(common.FIG_DIR, "hist_conf_min.pdf"),
        attempts,
        "conf_min",
        "Minimum word confidence of the extracted value",
    )


def week3(calib, held, names, macros):
    Xc, yc = common.matrix(calib, names)
    Xh, yh = common.matrix(held, names)
    gc = [a.get("speaker_id") or a["source"] for a in calib]
    gh = [a.get("speaker_id") or a["source"] for a in held]

    models = fit_all(calib, Xc, yc, names, gc)
    # Model selection on calibration out-of-fold log-loss (no held-out leakage)
    cv = {
        n: log_loss(yc, np.clip(oof, 1e-4, 1 - 1e-4)) for n, (_, oof) in models.items()
    }
    selected = min(cv, key=cv.get)

    raw = {
        "legacy_raw": np.array([a["features"]["legacy_conf"] for a in held]),
        "mean_raw": np.array([a["features"]["conf_mean"] for a in held]),
    }
    results = {name: metrics(yh, p, gh) for name, p in raw.items()}
    for name, (spec, _) in models.items():
        results[name] = {**metrics(yh, predict(spec, held), gh), "cv_nll": cv[name]}

    labels = {
        "legacy_raw": "Legacy score (raw)",
        "mean_raw": "Fixed mean conf. (raw)",
        "iso_mean": "Isotonic(mean)",
        "iso_min": "Isotonic(min)",
        "platt_mean": "Platt(mean)",
        "logistic_all_C0.1": "Logistic all ($C{=}0.1$)",
        "logistic_all_C1.0": "Logistic all ($C{=}1$)",
        "logistic_all_iso": "Logistic all + isotonic",
    }
    rows = []
    for name, r in results.items():
        mark = "\\textbf{" if name == selected else ""
        end = "}" if mark else ""
        rows.append(
            f"{mark}{labels[name]}{end} & {tex_num(r['auroc'])} [{tex_num(r['auroc_lo'])}, {tex_num(r['auroc_hi'])}]"
            f" & {tex_num(r['ece'], 3)} & {tex_num(r['brier'], 3)} \\\\"
        )
    with open(os.path.join(common.GEN_DIR, "calibration_table.tex"), "w") as f:
        f.write("\n".join(rows) + "\n")

    sel = results[selected]
    best_auroc = max(results[n]["auroc"] for n in models)
    macros.update(
        {
            "WThreeSelected": labels[selected],
            "WThreeAUROC": tex_num(sel["auroc"]),
            "WThreeAUROCLo": tex_num(sel["auroc_lo"]),
            "WThreeAUROCHi": tex_num(sel["auroc_hi"]),
            "WThreeECE": tex_num(sel["ece"], 3),
            "WThreeLegacyAUROC": tex_num(results["legacy_raw"]["auroc"]),
            "WThreeLegacyECE": tex_num(results["legacy_raw"]["ece"], 3),
            "WThreeMeanRawECE": tex_num(results["mean_raw"]["ece"], 3),
            "WThreeBestAUROC": tex_num(best_auroc),
            "HOneVerdict": "supported" if sel["auroc"] >= 0.80 else "not supported",
            "HTwoVerdict": "supported" if sel["ece"] <= 0.05 else "not supported",
        }
    )

    sel_p = predict(models[selected][0], held)
    curves = [
        ("Legacy score", yh, raw["legacy_raw"]),
        ("Fixed mean conf.", yh, raw["mean_raw"]),
        (labels[selected], yh, sel_p),
    ]
    reliability_plot(os.path.join(common.FIG_DIR, "reliability.pdf"), curves)
    roc_plot(os.path.join(common.FIG_DIR, "roc.pdf"), curves)

    with open(os.path.join(common.MODEL_DIR, "confidence_model.json"), "w") as f:
        json.dump(
            {**models[selected][0], "name": selected, "features_available": names},
            f,
            indent=2,
        )
    with open(os.path.join(common.MODEL_DIR, "all_models.json"), "w") as f:
        json.dump({n: s for n, (s, _) in models.items()}, f, indent=2)
    with open(os.path.join(common.MODEL_DIR, "calibration_metrics.json"), "w") as f:
        json.dump(
            {"selected": selected, "results": results}, f, indent=2, default=float
        )
    return selected, results


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--data", default=common.DATA_DIR)
    parser.add_argument("--task", default="hric")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    common.ensure_dirs()
    attempts = common.labeled_attempts(args.data, task=args.task)
    if len(attempts) < 20:
        raise SystemExit(
            f"Only {len(attempts)} labeled attempts in {args.data}, nothing to analyze"
        )
    synthetic = any(a.get("synthetic") for a in attempts)
    names = common.feature_names(attempts)
    calib, held, calib_sp, held_sp = common.speaker_split(attempts, seed=args.seed)

    tts = not synthetic and any(a.get("data_source") == "tts" for a in attempts)
    if synthetic:
        source = "synthetic (pipeline validation only)"
    elif tts:
        source = "TTS voices transcribed by the robot Whisper (proxy)"
    else:
        source = "real robot sessions"
    macros = {
        "DataSource": source,
        "IsSynthetic": "1" if synthetic else "0",
        "IsTTS": "1" if tts else "0",
        "FeatureList": ", ".join(
            f"\\texttt{{{n.replace('_', chr(92) + '_')}}}" for n in names
        ),
    }
    week1(attempts, macros)
    week2(attempts, calib, held, calib_sp, held_sp, macros)
    selected, results = week3(calib, held, names, macros)
    common.write_macros(os.path.join(common.GEN_DIR, "analysis.tex"), macros)

    print(f"Data: {args.data} ({macros['DataSource']})")
    print(
        f"Attempts: {len(attempts)} from {macros['WTwoSpeakers']} speakers, calib {len(calib)} / held-out {len(held)}"
    )
    print(
        f"Week 1: normalized values {macros['WOneNormalized']}, decisions changed by the fix {macros['WOneFlipped']}"
    )
    print(f"{'model':22s} {'AUROC':>6s} {'95% CI':>14s} {'ECE':>6s} {'Brier':>6s}")
    for name, r in results.items():
        star = "*" if name == selected else " "
        print(
            f"{star}{name:21s} {r['auroc']:6.3f} [{r['auroc_lo']:.2f}, {r['auroc_hi']:.2f}] {r['ece']:6.3f} {r['brier']:6.3f}"
        )
    print(
        f"H1 (AUROC >= 0.80): {macros['HOneVerdict']}; H2 (ECE <= 0.05): {macros['HTwoVerdict']}"
    )


if __name__ == "__main__":
    main()
