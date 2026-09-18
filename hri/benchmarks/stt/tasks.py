"""STT benchmark task definitions.

Supported transcription kwargs (forwarded to transcribe_file):
  language, vad, hotwords, initial_prompt
"""

import os

from benchmark_stt import (
    load_test_cases,
    run_accuracy,
    run_latency,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RECORDINGS_DIR = os.path.join(SCRIPT_DIR, "recordings")
TEST_CASES_FILE = os.path.join(SCRIPT_DIR, "test_cases.json")

_TRANSCRIBE_KEYS = {"language", "vad", "hotwords", "initial_prompt"}


def _extract_transcribe_kwargs(kwargs: dict) -> dict:
    return {k: v for k, v in kwargs.items() if k in _TRANSCRIBE_KEYS and v is not None}


class AccuracyTask:
    """Run all test cases and compute aggregate WER / accuracy."""

    name = "accuracy"

    @staticmethod
    def run(model: str, runs: int = 1, **kwargs) -> dict:
        test_cases = load_test_cases(TEST_CASES_FILE)
        tkwargs = _extract_transcribe_kwargs(kwargs)
        results = run_accuracy(model, test_cases, **tkwargs)
        passed = sum(1 for r in results if r["passed"])
        total = len(results)
        avg_wer = sum(r["wer"] for r in results) / total if total else 0

        return {
            "cases": [
                {
                    "input": r["name"],
                    "expected": r["expected"],
                    "got": r["actual"],
                    "passed": r["passed"],
                    "wer": r["wer"],
                }
                for r in results
            ],
            "accuracy": round(passed / total, 3) if total else 0,
            "avg_wer": round(avg_wer, 4),
            "total": total,
            "passed": passed,
        }


class LatencyTask:
    """Measure processing latency and RTF on a representative audio file."""

    name = "latency"

    @staticmethod
    def run(model: str, runs: int = 3, audio: str | None = None, **kwargs) -> dict:
        if audio is None:
            audio = os.path.join(RECORDINGS_DIR, "hello_frida.wav")
        if not os.path.isfile(audio):
            return {"error": f"audio not found: {audio}"}

        tkwargs = _extract_transcribe_kwargs(kwargs)
        lat = run_latency(audio, model, n_runs=runs, **tkwargs)
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
