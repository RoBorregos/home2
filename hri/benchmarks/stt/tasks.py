"""STT benchmark task definitions.

Each task class wraps a benchmark_stt function and exposes a standard
``run(model, runs)`` interface so the runner (run.sh / test_hri_manager)
can invoke any task uniformly.
"""

import json
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from benchmark_stt import (
    calculate_wer,
    load_test_cases,
    run_accuracy,
    run_latency,
    transcribe_file,
)

RECORDINGS_DIR = os.path.join(SCRIPT_DIR, "recordings")
TEST_CASES_FILE = os.path.join(SCRIPT_DIR, "test_cases.json")


class AccuracyTask:
    """Run all test cases and compute aggregate WER / accuracy."""

    name = "accuracy"

    @staticmethod
    def run(model: str, runs: int = 1, **kwargs) -> dict:
        test_cases = load_test_cases(TEST_CASES_FILE)
        results = run_accuracy(model, test_cases)
        passed = sum(1 for r in results if r["passed"])
        total = len(results)
        avg_wer = sum(r["wer"] for r in results) / total if total else 0
        avg_acc = sum(r["accuracy"] for r in results) / total if total else 0

        return {
            "cases": [
                {
                    "input": r["name"],
                    "expected": r["expected"],
                    "got": r["actual"],
                    "passed": r["passed"],
                    "wer": r["wer"],
                    "accuracy": r["accuracy"],
                }
                for r in results
            ],
            "accuracy": round(avg_acc, 3),
            "avg_wer": round(avg_wer, 4),
            "total": total,
            "passed": passed,
        }


class LatencyTask:
    """Measure processing latency and RTF on a representative audio file."""

    name = "latency"

    @staticmethod
    def run(model: str, runs: int = 3, audio: str = None, **kwargs) -> dict:
        if audio is None:
            audio = os.path.join(RECORDINGS_DIR, "hello_frida.wav")
        if not os.path.isfile(audio):
            return {"error": f"audio not found: {audio}"}

        lat = run_latency(audio, model, n_runs=runs)
        return {
            "audio_duration": lat["audio_duration"],
            "n_runs": lat["n_runs"],
            "avg_latency_s": lat["avg_latency_s"],
            "avg_rtf": lat["avg_rtf"],
            "throughput": lat["throughput"],
            "min_latency_s": lat["min_latency_s"],
            "max_latency_s": lat["max_latency_s"],
        }


TASK_REGISTRY = {
    "accuracy": AccuracyTask,
    "latency": LatencyTask,
}
