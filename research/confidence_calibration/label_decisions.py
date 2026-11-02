#!/usr/bin/env python3

"""
Label ask_and_confirm attempts as correct / incorrect (#1384).

Copy the robot logs first: logs/hri_decisions/*.jsonl -> research/confidence_calibration/data/logs/

    python label_decisions.py                # ask for every unlabeled attempt
    python label_decisions.py --review       # only attempts without intended value or where the
                                             # intended value and the user's yes/no disagree
    python label_decisions.py --stats

Labels are appended to data/labels.jsonl (the raw logs are never modified), so labeling can be
stopped (q) and resumed. Keys: y = correct, n = wrong, s = skip, q = quit.
"""

import argparse
import json
import os
from collections import Counter
from datetime import datetime

from common import DATA_DIR, DECIDED, auto_label, load_labels, load_logs, same_value


def needs_review(rec):
    if not rec.get("intended"):
        return True
    if rec.get("confirm_answer") in ("yes", "no"):
        by_intended = same_value(
            rec["extracted_value"], rec["intended"], rec.get("slot", "")
        )
        return by_intended != (rec["confirm_answer"] == "yes")
    return False


def show(rec, idx, total):
    print("\n" + "=" * 72)
    print(
        f"[{idx}/{total}] {rec['id']}  speaker={rec.get('speaker_id', '')}  {rec.get('condition', '')}"
    )
    print(f"  Q:            {rec.get('question')}")
    print(f"  heard:        {rec.get('transcription')}")
    if rec.get("words"):
        print(
            "  words:        "
            + " ".join(
                f"{w}({c:.2f})" for w, c in zip(rec["words"], rec["confidences"])
            )
        )
    print(f"  extracted:    \033[1m{rec.get('extracted_value')}\033[0m")
    if rec.get("intended"):
        print(f"  intended:     {rec['intended']}")
    if rec.get("confirm_answer"):
        print(f"  user said:    {rec['confirm_answer']}")
    print(f"  decision:     {rec.get('decision')}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--data", default=DATA_DIR)
    parser.add_argument("--review", action="store_true", help="only ambiguous attempts")
    parser.add_argument(
        "--relabel", action="store_true", help="also show already labeled attempts"
    )
    parser.add_argument("--stats", action="store_true")
    args = parser.parse_args()

    records = [
        r
        for r in load_logs(args.data)
        if r.get("decision") in DECIDED and r.get("extracted_value")
    ]
    labels = load_labels(args.data)

    if args.stats:
        sources = Counter()
        for rec in records:
            if rec["id"] in labels:
                sources["human"] += 1
            else:
                sources[auto_label(rec)[1] or "unlabeled"] += 1
        print(f"{len(records)} attempts:", dict(sources))
        return

    todo = [r for r in records if args.relabel or r["id"] not in labels]
    if args.review:
        todo = [r for r in todo if needs_review(r)]
    print(f"{len(todo)} attempts to label")

    path = os.path.join(args.data, "labels.jsonl")
    with open(path, "a", encoding="utf-8") as out:
        for idx, rec in enumerate(todo, 1):
            show(rec, idx, len(todo))
            guess = auto_label(rec)[0]
            hint = {True: " [Y/n/s/q]", False: " [y/N/s/q]", None: " [y/n/s/q]"}[guess]
            while True:
                key = input("  correct?" + hint + " ").strip().lower() or {
                    True: "y",
                    False: "n",
                }.get(guess, "")
                if key in ("y", "n", "s", "q"):
                    break
            if key == "q":
                break
            if key == "s":
                continue
            out.write(
                json.dumps(
                    {
                        "id": rec["id"],
                        "correct": key == "y",
                        "labeled_at": datetime.now().isoformat(),
                    }
                )
                + "\n"
            )
            out.flush()
    print(f"Labels saved to {path}")


if __name__ == "__main__":
    main()
