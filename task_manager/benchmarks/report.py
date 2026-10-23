#!/usr/bin/env python3
"""
Summarize the HRI experiments and export every participant to one CSV.

python3 report.py                       # all experiments in results/
python3 report.py --experiment exp2_schedule --label pilot
python3 report.py --csv all_sessions.csv
"""

import argparse
import csv
import glob
import json
import os
import statistics
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
SCALAR_FIELDS = (
    "fields_captured",
    "id_attempts",
    "exact",
    "resolution",
    "questions_answered",
    "turns",
    "ended_by",
    "scripted",
    "error",
)


def percentile(values, p):
    values = sorted(values)
    k = (len(values) - 1) * p / 100
    low, high = int(k), min(int(k) + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (k - low)


def stats(values):
    values = [v for v in values if isinstance(v, (int, float)) and not isinstance(v, bool)]
    if not values:
        return "-"
    return (
        f"mean {statistics.mean(values):6.2f}  p50 {percentile(values, 50):6.2f}  "
        f"p95 {percentile(values, 95):6.2f}  (n={len(values)})"
    )


def rate(flags):
    flags = [f for f in flags if f is not None]
    return f"{100 * sum(flags) / len(flags):5.1f}% (n={len(flags)})" if flags else "-"


def load(paths):
    sessions = []
    for path in paths:
        with open(path) as file:
            sessions.extend(json.loads(line) for line in file if line.strip())
    return sessions


def robot_text(session) -> str:
    return " ".join(t["text"] for t in session.get("transcripts", []) if t["role"] == "robot")


def tool_names(session) -> list[str]:
    return [c["name"] for turn in session.get("turns_log", []) for c in turn["tool_calls"]]


def expectation_passed(session):
    """Scripted runs only: did the robot say the expected facts and call the tools?"""
    expect = session.get("expect")
    if not expect:
        return None
    text, tools = robot_text(session), tool_names(session)
    return (
        all(m.lower() in text.lower() for m in expect.get("mentions", []))
        and all(t in tools for t in expect.get("tools", []))
        and not any(f in text for f in expect.get("forbidden", []))
    )


def report(name, sessions):
    print(f"\n=== {name}: {len(sessions)} sessions ===")
    outcomes = Counter(s["outcome"] for s in sessions)
    print("Outcomes:            " + ", ".join(f"{k}={v}" for k, v in outcomes.most_common()))
    print("Success rate:        " + rate([s["outcome"] == "success" for s in sessions]))
    print("Total time (s):      " + stats([s.get("duration_s") for s in sessions]))
    print("STT confidence:      " + stats([s.get("stt_confidence_avg") for s in sessions]))
    sources = Counter()
    for s in sessions:
        sources.update(s.get("data_sources") or {})
    if sources:
        print("Data sources:        " + ", ".join(f"{k}={v}" for k, v in sources.items()))
    expectations = [expectation_passed(s) for s in sessions]
    if any(e is not None for e in expectations):
        print("Scripted expectations: " + rate(expectations))

    for field in SCALAR_FIELDS:
        values = [s.get(field) for s in sessions if s.get(field) is not None]
        if not values:
            continue
        if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in values):
            print(f"{field + ':':<21}{stats(values)}")
        else:
            counts = Counter(str(v) for v in values)
            print(f"{field + ':':<21}" + ", ".join(f"{k}={v}" for k, v in counts.items()))

    intents = Counter(
        q.get("intent") or "unknown" for s in sessions for q in s.get("questions", [])
    )
    if intents:
        print("Intents:             " + ", ".join(f"{k}={v}" for k, v in intents.most_common()))

    turns = [t for s in sessions for t in s.get("turns_log", [])]
    if turns:
        tools = Counter(c["name"] for t in turns for c in t["tool_calls"])
        failed = sum(not c["ok"] for t in turns for c in t["tool_calls"])
        print("Agent latency/turn:  " + stats([t["agent_latency_s"] for t in turns]))
        print("Tool calls/turn:     " + stats([len(t["tool_calls"]) for t in turns]))
        print("Tools used:          " + ", ".join(f"{k}={v}" for k, v in tools.most_common()))
        print(f"Failed tool calls:   {failed}")

    faces = [s["face_following"] for s in sessions if s.get("face_following")]
    if faces:
        print("\nFollow face:")
        print("  Person detected:   " + rate([s.get("person_detected") for s in sessions]))
        print("  Tracked ratio:     " + stats([f["tracked_ratio"] for f in faces]))
        print("  Follow rate (Hz):  " + stats([f["rate_hz"] for f in faces]))
        print("  Max gap (s):       " + stats([f["max_gap_s"] for f in faces]))

    print("\nPhase times (s):")
    phases = defaultdict(list)
    for s in sessions:
        for phase, value in s.get("phases", {}).items():
            phases[phase].append(value)
    for phase in sorted(phases):
        print(f"  {phase:<18} {stats(phases[phase])}")

    print("\nCalls per session (nested included) and latency per call:")
    for method in sorted({m for s in sessions for m in s.get("calls", {})}):
        counts = [s.get("calls", {}).get(method, {}).get("count", 0) for s in sessions]
        latencies = [
            s["calls"][method]["total_s"] / s["calls"][method]["count"]
            for s in sessions
            if s.get("calls", {}).get(method, {}).get("count")
        ]
        print(f"  {method:<20} count {stats(counts)}")
        print(f"  {'':<20} s/call {stats(latencies)}")

    surveys = [s["survey"] for s in sessions if s.get("survey")]
    if surveys:
        print("\nSurvey (1-5):")
        print("  Completed:         " + rate([sv.get("completed") for sv in surveys]))
        questions = sorted({q for sv in surveys for q in sv.get("answers", {})})
        for q in questions:
            values = [sv["answers"].get(q) for sv in surveys if q in sv.get("answers", {})]
            if all(isinstance(v, (int, float)) for v in values):
                print(f"  {q:<18} {stats(values)}")
            else:
                counts = Counter(str(v) for v in values)
                print(f"  {q:<18} " + ", ".join(f"{k}={v}" for k, v in counts.items()))


def flatten(session) -> dict:
    row = {
        k: session.get(k)
        for k in ("experiment", "label", "session", "started_at", "outcome", "duration_s")
    }
    row["stt_confidence_avg"] = session.get("stt_confidence_avg")
    row["person_detected"] = session.get("person_detected")
    face = session.get("face_following") or {}
    row["face_tracked_ratio"] = face.get("tracked_ratio")
    row["face_max_gap_s"] = face.get("max_gap_s")
    for source, count in (session.get("data_sources") or {}).items():
        row[f"source_{source}"] = count
    for field in SCALAR_FIELDS:
        row[field] = session.get(field)
    row["expectation_passed"] = expectation_passed(session)
    for phase, value in session.get("phases", {}).items():
        row[f"phase_{phase}_s"] = value
    for method, call in session.get("calls", {}).items():
        row[f"calls_{method}"] = call["count"]
    turns = session.get("turns_log", [])
    if turns:
        row["agent_latency_mean_s"] = round(statistics.mean(t["agent_latency_s"] for t in turns), 3)
        row["tool_calls"] = sum(len(t["tool_calls"]) for t in turns)
    survey = session.get("survey") or {}
    row["survey_completed"] = survey.get("completed")
    for question, answer in (survey.get("answers") or {}).items():
        row[f"survey_{question}"] = answer
    row["transcript"] = " | ".join(
        f"{t['role']}: {t['text']}" for t in session.get("transcripts", [])
    )
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="*")
    parser.add_argument("--experiment")
    parser.add_argument("--label")
    parser.add_argument("--csv", help="Write every session to this CSV file")
    args = parser.parse_args()

    files = args.files or sorted(glob.glob(os.path.join(HERE, "results", "*", "*.jsonl")))
    sessions = load(files)
    if args.experiment:
        sessions = [s for s in sessions if s.get("experiment") == args.experiment]
    if args.label:
        sessions = [s for s in sessions if s.get("label") == args.label]
    if not sessions:
        print("No sessions found.")
        return

    groups = defaultdict(list)
    for s in sessions:
        groups[f"{s.get('experiment')} / {s.get('label')}"].append(s)
    for name, group in sorted(groups.items()):
        report(name, group)

    if args.csv:
        rows = [flatten(s) for s in sessions]
        columns = list(dict.fromkeys(k for row in rows for k in row))
        with open(args.csv, "w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nWrote {len(rows)} sessions to {args.csv}")


if __name__ == "__main__":
    main()
