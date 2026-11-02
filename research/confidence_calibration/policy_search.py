#!/usr/bin/env python3

"""
Threshold search and offline policy comparison (#1384, week 3). Run after analyze.py.

    python policy_search.py --data data/synthetic

Each dialog slot (one ask_and_confirm call) is replayed under a policy:
    accept  -> value stored; wrong value = penalty
    confirm -> non-essential question; the user's logged yes/no decides (no -> next attempt)
    repeat  -> free "could you repeat?", next attempt (last attempt confirms instead, as on the robot)
The next attempt is the next logged attempt of that slot or, when the log has none (the user
accepted), a random attempt of the same speaker and slot type (cluster bootstrap). This
counterfactual is the main limitation of the offline evaluation.

Scoring per slot (HRIC 2027 Rev-1 scoresheet, see paper): +15 when no confirmation question was
asked, -40 for a wrong memorized value, -40 (assumption) when no value is obtained.

Outputs: paper/generated/policy.tex, paper/generated/policy_table.tex, paper/figures/tradeoff.pdf,
models/thresholds.json
"""

import argparse
import json
import os
import random
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import common  # noqa: E402
from analyze import AQUA, BLUE, INK_2, ORANGE, pct, tex_num  # noqa: E402
from common import cp  # noqa: E402

COSTS = {
    # bonus: no-confirmation bonus per slot, wrong / missing: memorized value penalties
    "hric": {"bonus": 15, "wrong": -40, "missing": -40, "repeat": 0, "confirm": 0},
    # GPSR: rephrase request -30, a misunderstood command loses understanding (80) + solving (250)
    "gpsr": {"bonus": 0, "wrong": -330, "missing": -330, "repeat": -30, "confirm": 0},
}
SECONDS = {"ask": 6.0, "confirm": 5.0, "repeat": 5.0}
MAX_WRONG_INCREASE = 0.02


class Simulator:
    def __init__(self, attempts, model, retries=5, sims=30, seed=0):
        self.slots = common.slots(attempts)
        self.retries = retries
        self.sims = sims
        self.seed = seed
        self.pool = defaultdict(list)
        for a in attempts:
            self.pool[
                (a.get("speaker_id") or a["source"], a.get("slot") or a.get("query"))
            ].append(a)
        for a in attempts:
            a["p"] = model.predict(a["features"]) if model else None

    @staticmethod
    def user_says_yes(a):
        return (
            a["confirm_answer"] == "yes"
            if a.get("confirm_answer") in ("yes", "no")
            else a["correct"]
        )

    def run_slot(self, slot, decide, rng):
        key = (
            slot[0].get("speaker_id") or slot[0]["source"],
            slot[0].get("slot") or slot[0].get("query"),
        )
        out = {
            "confirms": 0,
            "repeats": 0,
            "turns": 0,
            "seconds": 0.0,
            "wrong": 0,
            "missing": 0,
        }
        for i in range(self.retries):
            a = slot[i] if i < len(slot) else rng.choice(self.pool[key])
            out["turns"] += 1
            out["seconds"] += SECONDS["ask"]
            decision = decide(a)
            if decision == cp.REPEAT and i == self.retries - 1:
                decision = cp.CONFIRM
            if decision == cp.ACCEPT:
                out["wrong"] = int(not a["correct"])
                return out
            if decision == cp.REPEAT:
                out["repeats"] += 1
                out["seconds"] += SECONDS["repeat"]
                continue
            out["confirms"] += 1
            out["turns"] += 1
            out["seconds"] += SECONDS["confirm"]
            if self.user_says_yes(a):
                out["wrong"] = int(not a["correct"])
                return out
        out["missing"] = 1
        return out

    def evaluate(self, decide, costs):
        totals = defaultdict(float)
        n = 0
        for s in range(self.sims):
            rng = random.Random(self.seed + s)
            for slot in self.slots:
                out = self.run_slot(slot, decide, rng)
                for k, v in out.items():
                    totals[k] += v
                totals["score"] += (
                    (costs["bonus"] if out["confirms"] == 0 else 0)
                    + costs["wrong"] * out["wrong"]
                    + costs["missing"] * out["missing"]
                    + costs["repeat"] * out["repeats"]
                    + costs["confirm"] * out["confirms"]
                )
                n += 1
        return {k: v / n for k, v in totals.items()}


def policies(thresholds_binary=None, thresholds_three=None):
    p = {
        "always_confirm": lambda a: cp.CONFIRM,
        "legacy": lambda a: cp.decide(a["features"], "legacy", fix_matching=False)[0],
        "legacy_fixed": lambda a: cp.decide(a["features"], "legacy", fix_matching=True)[
            0
        ],
    }
    if thresholds_binary:
        t = thresholds_binary
        p["binary"] = lambda a: cp.ACCEPT if a["p"] >= t["accept"] else cp.CONFIRM
    if thresholds_three:
        t = thresholds_three
        p["three_way"] = lambda a: (
            cp.ACCEPT
            if a["p"] >= t["accept"]
            else cp.REPEAT
            if a["p"] < t["repeat"]
            else cp.CONFIRM
        )
    return p


def search(sim, costs, max_wrong):
    grid = np.round(np.arange(0.50, 1.0001, 0.02), 3)
    best_binary, best_three, curve = None, None, []
    for acc in grid:
        r = sim.evaluate(
            lambda a, acc=acc: cp.ACCEPT if a["p"] >= acc else cp.CONFIRM, costs
        )
        curve.append((acc, r))
        if r["wrong"] <= max_wrong and (
            best_binary is None or r["score"] > best_binary[1]["score"]
        ):
            best_binary = ({"accept": float(acc)}, r)
        for rep in np.round(np.arange(0.0, acc, 0.05), 3):
            r3 = sim.evaluate(
                lambda a, acc=acc, rep=rep: cp.ACCEPT
                if a["p"] >= acc
                else cp.REPEAT
                if a["p"] < rep
                else cp.CONFIRM,
                costs,
            )
            if r3["wrong"] <= max_wrong and (
                best_three is None or r3["score"] > best_three[1]["score"]
            ):
                best_three = ({"accept": float(acc), "repeat": float(rep)}, r3)
    return best_binary, best_three, curve


def tradeoff_plot(path, curve, points):
    fig, ax = plt.subplots(figsize=(3.4, 2.7))
    ax.plot(
        [r["confirms"] for _, r in curve],
        [100 * r["wrong"] for _, r in curve],
        color=BLUE,
        lw=2,
        label=r"Calibrated binary, sweep $\tau_{accept}$",
    )
    styles = {
        "always_confirm": ("o", INK_2),
        "legacy": ("s", ORANGE),
        "legacy_fixed": ("^", ORANGE),
        "binary": ("D", BLUE),
        "three_way": ("*", AQUA),
    }
    for name, r in points.items():
        marker, color = styles[name]
        ax.plot(
            r["confirms"],
            100 * r["wrong"],
            marker=marker,
            color=color,
            ms=7 if marker != "*" else 11,
            markeredgecolor="white",
            markeredgewidth=0.8,
            ls="none",
            label=LABELS[name],
        )
    ax.set_xlabel("Confirmation questions per slot")
    ax.set_ylabel("Wrong memorized values (%)")
    ax.legend(fontsize=6, loc="upper right")
    fig.savefig(path)
    fig.savefig(path.replace(".pdf", ".png"))
    plt.close(fig)


LABELS = {
    "always_confirm": "(a) Always confirm",
    "legacy": "(b) Current heuristic 0.7/0.7",
    "legacy_fixed": "(b') Heuristic + matching fix",
    "binary": "(c) Calibrated binary",
    "three_way": "(d) Calibrated three-way",
}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--data", default=common.DATA_DIR)
    parser.add_argument("--task", default="hric", choices=list(COSTS))
    parser.add_argument(
        "--model", default=os.path.join(common.MODEL_DIR, "confidence_model.json")
    )
    parser.add_argument(
        "--retries", type=int, default=5, help="hric_task_manager uses retries=5"
    )
    parser.add_argument("--sims", type=int, default=30)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--max-wrong",
        default="legacy+2pp",
        choices=["legacy", "legacy+2pp", "always+2pp"],
        help="wrong-value constraint for the threshold search, measured on the calibration split",
    )
    args = parser.parse_args()

    common.ensure_dirs()
    costs = COSTS[args.task]
    model = cp.CalibratedModel.load(args.model)
    attempts = common.labeled_attempts(args.data, task=args.task)
    calib, held, _, _ = common.speaker_split(attempts, seed=args.seed)
    sim_c = Simulator(calib, model, args.retries, args.sims, args.seed)
    sim_h = Simulator(held, model, args.retries, args.sims, args.seed)

    reference, margin = {
        "legacy": ("legacy", 0.0),
        "legacy+2pp": ("legacy", MAX_WRONG_INCREASE),
        "always+2pp": ("always_confirm", MAX_WRONG_INCREASE),
    }[args.max_wrong]
    max_wrong = sim_c.evaluate(policies()[reference], costs)["wrong"] + margin
    best_binary, best_three, curve = search(sim_c, costs, max_wrong)
    held_results = {
        name: sim_h.evaluate(fn, costs)
        for name, fn in policies(
            best_binary[0] if best_binary else None,
            best_three[0] if best_three else None,
        ).items()
    }
    held_curve = [
        (
            acc,
            sim_h.evaluate(
                lambda a, acc=acc: cp.ACCEPT if a["p"] >= acc else cp.CONFIRM, costs
            ),
        )
        for acc, _ in curve
    ]

    rows = []
    for name, r in held_results.items():
        rows.append(
            f"{LABELS[name]} & {pct(r['wrong'])} & {tex_num(2 * r['confirms'])} & {tex_num(2 * r['repeats'])}"
            f" & {tex_num(r['turns'])} & {tex_num(r['seconds'], 1)} & {pct(r['missing'])} & {tex_num(2 * r['score'], 1)} \\\\"
        )
    with open(os.path.join(common.GEN_DIR, "policy_table.tex"), "w") as f:
        f.write("\n".join(rows) + "\n")
    tradeoff_plot(
        os.path.join(common.FIG_DIR, "tradeoff.pdf"), held_curve, held_results
    )

    legacy, best = held_results["legacy"], held_results.get("three_way")
    macros = {
        "PolicyCostBonus": costs["bonus"],
        "PolicyCostWrong": costs["wrong"],
        "PolicyCostMissing": costs["missing"],
        "PolicyRetries": args.retries,
        "PolicySims": args.sims,
        "PolicyMaxWrong": pct(max_wrong),
        "PolicyMaxWrongRule": {
            "legacy": "the current heuristic's wrong-value rate",
            "legacy+2pp": "the current heuristic's wrong-value rate plus 2 percentage points",
            "always+2pp": "the always-confirm wrong-value rate plus 2 percentage points",
        }[args.max_wrong],
    }
    if best_binary:
        macros["TauBinaryAccept"] = tex_num(best_binary[0]["accept"])
    if best_three:
        macros["TauAccept"] = tex_num(best_three[0]["accept"])
        macros["TauRepeat"] = tex_num(best_three[0]["repeat"])
    if best:
        reduction = (
            1 - best["confirms"] / legacy["confirms"] if legacy["confirms"] else 0.0
        )
        wrong_delta = best["wrong"] - legacy["wrong"]
        h3 = (
            reduction >= 0.5 and wrong_delta <= 0.02 and best["score"] > legacy["score"]
        )
        macros.update(
            {
                "HThreeConfirmReduction": pct(reduction)
                if reduction >= 0
                else "$-$" + pct(-reduction),
                "HThreeWrongDelta": f"{100 * wrong_delta:+.1f} pp",
                "HThreeScoreLegacy": tex_num(2 * legacy["score"], 1),
                "HThreeScoreBest": tex_num(2 * best["score"], 1),
                "HThreeScoreAlways": tex_num(
                    2 * held_results["always_confirm"]["score"], 1
                ),
                "HThreeVerdict": "supported" if h3 else "not supported",
                "HThreeA": "yes" if reduction >= 0.5 else "no",
                "HThreeB": "yes" if wrong_delta <= 0.02 else "no",
                "HThreeC": "yes" if best["score"] > legacy["score"] else "no",
            }
        )
    common.write_macros(os.path.join(common.GEN_DIR, "policy.tex"), macros)
    with open(os.path.join(common.MODEL_DIR, "thresholds.json"), "w") as f:
        json.dump(
            {
                args.task: {
                    "binary": best_binary[0] if best_binary else None,
                    "three_way": best_three[0] if best_three else None,
                },
                "held_out": held_results,
            },
            f,
            indent=2,
        )

    print(
        f"Thresholds ({args.task}): binary {best_binary and best_binary[0]}, three-way {best_three and best_three[0]}"
    )
    print(
        f"{'policy':16s} {'wrong':>6s} {'conf/g':>6s} {'rep/g':>6s} {'turns':>6s} {'sec':>5s} {'miss':>5s} {'pts/g':>6s}"
    )
    for name, r in held_results.items():
        print(
            f"{name:16s} {100 * r['wrong']:5.1f}% {2 * r['confirms']:6.2f} {2 * r['repeats']:6.2f} "
            f"{r['turns']:6.2f} {r['seconds']:5.1f} {100 * r['missing']:4.1f}% {2 * r['score']:6.1f}"
        )
    if best:
        print(
            f"Max wrong rate (calibration split, {args.max_wrong}): {100 * max_wrong:.1f}%"
        )
        print(
            f"H3: confirmation reduction {100 * reduction:.1f}%, wrong {macros['HThreeWrongDelta']}, "
            f"score {macros['HThreeScoreLegacy']} -> {macros['HThreeScoreBest']}: {macros['HThreeVerdict']}"
        )


if __name__ == "__main__":
    main()
