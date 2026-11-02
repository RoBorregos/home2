#!/usr/bin/env python3

"""
SYNTHETIC decision logs to validate the analysis pipeline before real data exists (#1384).

The records have exactly the format written by HRITasks.ask_and_confirm and follow the collection
protocol (always_confirm=True, intended value known). Whisper word probabilities are simulated as
overconfident (wrong words still get ~0.65), some drinks are normalized by the "LLM"
("coke" -> "Coca-Cola") and a few extraction errors keep high STT confidence. None of the numbers
produced from this data are results: they only show that the pipeline works end to end.

    python make_synthetic.py --out data/synthetic --speakers 14 --trials 10
"""

import argparse
import json
import math
import os
import random

from common import cp

NAMES = [
    "Adel",
    "Angel",
    "Axel",
    "Charlie",
    "Jane",
    "Jules",
    "Morgan",
    "Paris",
    "Robin",
    "Simone",
]
CONFUSIONS = {
    "Adel": "Adele",
    "Angel": "Angela",
    "Axel": "Excel",
    "Charlie": "Carly",
    "Jane": "Jean",
    "Jules": "Jewels",
    "Morgan": "Megan",
    "Paris": "Harris",
    "Robin": "Robbie",
    "Simone": "Simon",
}
OUT_NAMES = ["Mohammed", "Ximena", "Siddharth", "Guadalupe", "Oluwaseun", "Bjorn"]
DRINKS = [
    "water",
    "fanta",
    "lemonade",
    "pepsi",
    "orange juice",
    "milk",
    "iced tea",
    "red bull",
]
NORMALIZED = {"coke": "Coca-Cola", "sidral": "Sidral Mundet", "oj": "orange juice"}
OUT_DRINKS = ["kombucha", "horchata", "ginger ale"]
WRONG_DRINKS = {
    "water": "vodka",
    "fanta": "panta",
    "milk": "mint",
    "pepsi": "pepper",
    "lemonade": "lemon",
    "iced tea": "ice",
    "red bull": "red ball",
}

TEMPLATES = {
    "name": ["my name is {}", "I'm {}", "{}", "hi I am {}", "call me {}"],
    "drink": [
        "I like {}",
        "my favorite drink is {}",
        "{} please",
        "{}",
        "I would like {}",
    ],
}


def sigmoid(x):
    return 1 / (1 + math.exp(-x))


def beta(rng, mean, strength):
    mean = min(max(mean, 0.02), 0.98)
    return rng.betavariate(mean * strength, (1 - mean) * strength)


def make_speaker(rng, idx):
    return {
        "speaker_id": f"syn{idx:02d}",
        "non_native": rng.random() < 0.5,
        "noise": rng.choice([0, 0, 1, 2]),  # none, music, crowd
        "distance": rng.choice([0.5, 1.5, 3.0]),
        "skill": rng.gauss(0, 0.5),
    }


def condition(sp):
    return ",".join(
        [
            "non-native" if sp["non_native"] else "native",
            ["quiet", "music", "crowd"][sp["noise"]],
            f"{sp['distance']}m",
        ]
    )


def pick_value(rng, slot):
    out_of_list = rng.random() < 0.25
    if slot == "name":
        return rng.choice(OUT_NAMES if out_of_list else NAMES), out_of_list, None
    if out_of_list:
        return rng.choice(OUT_DRINKS), True, None
    if rng.random() < 0.3:
        spoken, value = rng.choice(list(NORMALIZED.items()))
        return value, False, spoken
    return rng.choice(DRINKS), False, None


def attempt(rng, sp, slot, intended, out_of_list, spoken):
    difficulty = 0.8 * sp["non_native"] + 0.6 * sp["noise"] + 0.35 * sp["distance"]
    p_stt_ok = sigmoid(3.0 - difficulty + sp["skill"] - (1.3 if out_of_list else 0))
    stt_ok = rng.random() < p_stt_ok

    said = spoken or intended
    if stt_ok:
        heard_value = said
    elif slot == "name":
        heard_value = CONFUSIONS.get(said, said[:-1] + "a")
    else:
        heard_value = WRONG_DRINKS.get(said, said + "s")
    transcription = rng.choice(TEMPLATES[slot]).format(heard_value)
    words = transcription.split()
    value_words = set(cp._normalize(heard_value).split())

    confidences = []
    for w in words:
        if cp._normalize(w) in value_words:
            # Overconfident Whisper: wrong words still get high probabilities
            mean = 0.93 - 0.08 * difficulty if stt_ok else 0.68 - 0.05 * difficulty
            confidences.append(round(beta(rng, mean, 9), 3))
        else:
            confidences.append(round(beta(rng, 0.96 - 0.03 * difficulty, 20), 3))

    # LLM extraction: normalizes aliases, rarely picks a wrong value with confident STT
    if spoken and stt_ok:
        extracted = intended
    elif rng.random() < 0.03:
        extracted = rng.choice(NAMES if slot == "name" else DRINKS)
    else:
        extracted = heard_value.title() if slot == "name" else heard_value

    mean_conf = sum(confidences) / len(confidences)
    stt_meta = {
        "avg_logprob": round(math.log(mean_conf) - 0.15 + rng.gauss(0, 0.08), 3),
        "no_speech_prob": round(beta(rng, 0.03 + 0.05 * sp["noise"], 15), 3),
    }
    features = cp.compute_features(
        extracted, transcription, words, confidences, stt_meta=stt_meta
    )
    return transcription, words, confidences, stt_meta, extracted, features


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--out", default=os.path.join(os.path.dirname(__file__), "data", "synthetic")
    )
    parser.add_argument("--speakers", type=int, default=14)
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed", type=int, default=1384)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    os.makedirs(os.path.join(args.out, "logs"), exist_ok=True)
    total = 0
    for idx in range(args.speakers):
        sp = make_speaker(rng, idx)
        path = os.path.join(args.out, "logs", f"hric_{sp['speaker_id']}.jsonl")
        with open(path, "w", encoding="utf-8") as f:
            for trial in range(args.trials):
                for slot, query, question in (
                    ("name", "name", "What is your name?"),
                    ("drink", "LLM_drink", "What is your favorite drink?"),
                ):
                    intended, out_of_list, spoken = pick_value(rng, slot)
                    for attempt_idx in range(1, 4):
                        transcription, words, confs, meta, extracted, features = (
                            attempt(rng, sp, slot, intended, out_of_list, spoken)
                        )
                        correct = extracted.lower() == intended.lower()
                        # Users occasionally say yes to a wrong value
                        answer = "yes" if correct or rng.random() < 0.03 else "no"
                        record = {
                            "timestamp": f"synthetic-{sp['speaker_id']}-{trial}-{slot}-{attempt_idx}",
                            "speaker_id": sp["speaker_id"],
                            "condition": condition(sp),
                            "synthetic": True,
                            "slot": slot,
                            "intended": intended,
                            "out_of_list": out_of_list,
                            "trial": trial,
                            "task": "hric",
                            "policy": "legacy",
                            "question": question,
                            "query": query,
                            "transcription": transcription,
                            "extracted_value": extracted,
                            "words": words,
                            "confidences": confs,
                            "stt_meta": meta,
                            "options": None,
                            "features": features,
                            "decision": cp.CONFIRM,
                            "p_correct": None,
                            "confirm_answer": answer,
                            "attempt_idx": attempt_idx,
                        }
                        f.write(json.dumps(record) + "\n")
                        total += 1
                        if answer == "yes":
                            break
    print(
        f"Wrote {total} synthetic attempts from {args.speakers} speakers to {args.out}/logs"
    )


if __name__ == "__main__":
    main()
