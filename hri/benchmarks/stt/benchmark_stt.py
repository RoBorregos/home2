#!/usr/bin/env python3
"""
STT benchmark — accuracy (WER) and latency (RTF) for faster-whisper models.

Usage:
    python benchmark_stt.py --accuracy                             # run all test cases, report WER
    python benchmark_stt.py --accuracy --model base.en             # specific model
    python benchmark_stt.py --latency --audio recordings/hello_frida.wav  # single-file RTF
    python benchmark_stt.py --latency --runs 5                     # custom run count
    python benchmark_stt.py --batch recordings/                    # transcribe dir, print results
"""

import argparse
import csv
import json
import os
import re
import sys
import time
from datetime import datetime

import numpy as np
from faster_whisper import WhisperModel

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RECORDINGS_DIR = os.path.join(SCRIPT_DIR, "recordings")
TEST_CASES_FILE = os.path.join(SCRIPT_DIR, "test_cases.json")
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")

_model_cache: dict[str, WhisperModel] = {}


def _get_model(model_name: str) -> WhisperModel:
    if model_name not in _model_cache:
        from device_utils import detect_device_and_compute_type

        device, compute_type = detect_device_and_compute_type()
        print(f"Loading model '{model_name}' on {device} ({compute_type}) ...")
        _model_cache[model_name] = WhisperModel(
            model_name, device=device, compute_type=compute_type
        )
        print("Model loaded.\n")
    return _model_cache[model_name]


# ── WER ──────────────────────────────────────────────────────────────────────


def _remove_punctuation(text: str) -> str:
    return re.sub(r"[^\w\s]", "", text)


def calculate_wer(reference: str, hypothesis: str) -> float:
    """Word Error Rate via Levenshtein distance on word sequences."""
    ref_words = _remove_punctuation(reference).lower().split()
    hyp_words = _remove_punctuation(hypothesis).lower().split()
    n = len(ref_words)
    if n == 0:
        return 1.0 if len(hyp_words) > 0 else 0.0

    d = [[0] * (len(hyp_words) + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        d[i][0] = i
    for j in range(len(hyp_words) + 1):
        d[0][j] = j

    for i in range(1, n + 1):
        for j in range(1, len(hyp_words) + 1):
            if ref_words[i - 1] == hyp_words[j - 1]:
                d[i][j] = d[i - 1][j - 1]
            else:
                d[i][j] = min(d[i - 1][j - 1] + 1, d[i][j - 1] + 1, d[i - 1][j] + 1)

    return d[n][len(hyp_words)] / n


# ── Transcription ────────────────────────────────────────────────────────────


def transcribe_file(
    audio_path: str,
    model_name: str = "base.en",
    language: str = "en",
    vad: bool = True,
    hotwords: str = "",
    initial_prompt: str = "",
) -> dict:
    model = _get_model(model_name)

    t0 = time.time()
    segments, info = model.transcribe(
        audio_path,
        language=language,
        vad_filter=vad,
        word_timestamps=True,
        hotwords=hotwords or None,
        initial_prompt=initial_prompt or None,
    )

    all_segments = []
    full_text_parts = []
    for seg in segments or []:
        words = []
        if seg.words:
            words = [
                {
                    "word": w.word,
                    "confidence": round(w.probability, 4),
                    "start": round(w.start, 3),
                    "end": round(w.end, 3),
                }
                for w in seg.words
            ]
        all_segments.append(
            {
                "id": seg.id,
                "start": round(seg.start, 3),
                "end": round(seg.end, 3),
                "text": seg.text,
                "avg_logprob": round(seg.avg_logprob, 4),
                "no_speech_prob": round(seg.no_speech_prob, 4),
                "words": words,
            }
        )
        full_text_parts.append(seg.text)

    elapsed = time.time() - t0
    duration = info.duration if info else 0.0
    full_text = "".join(full_text_parts).strip()

    return {
        "text": full_text,
        "segments": all_segments,
        "language": info.language if info else "unknown",
        "language_probability": round(info.language_probability, 4) if info else 0,
        "audio_duration": round(duration, 3),
        "processing_time": round(elapsed, 3),
        "real_time_factor": round(elapsed / duration, 4) if duration > 0 else None,
    }


# ── Accuracy benchmark ───────────────────────────────────────────────────────


def load_test_cases(path: str = TEST_CASES_FILE) -> list[dict]:
    with open(path, "r") as f:
        return json.load(f)


def run_accuracy(
    model_name: str,
    test_cases: list[dict],
    audio_dir: str = RECORDINGS_DIR,
) -> list[dict]:
    results = []
    for tc in test_cases:
        audio_path = os.path.join(audio_dir, tc["audio_file"])
        expected = tc["expected_transcript"]

        if not os.path.exists(audio_path):
            results.append(
                {
                    "name": tc["name"],
                    "expected": expected,
                    "actual": "FILE_NOT_FOUND",
                    "wer": 1.0,
                    "accuracy": 0.0,
                    "passed": False,
                }
            )
            continue

        result = transcribe_file(audio_path, model_name=model_name)
        actual = result["text"]
        wer = calculate_wer(expected, actual)
        accuracy = 1.0 - wer
        passed = accuracy >= 0.8

        results.append(
            {
                "name": tc["name"],
                "expected": expected,
                "actual": actual,
                "wer": round(wer, 4),
                "accuracy": round(accuracy, 4),
                "rtf": result["real_time_factor"],
                "processing_time": result["processing_time"],
                "audio_duration": result["audio_duration"],
                "passed": passed,
            }
        )
    return results


# ── Latency benchmark ────────────────────────────────────────────────────────


def run_latency(
    audio_path: str,
    model_name: str,
    n_runs: int = 3,
) -> dict:
    latencies = []
    rtf_values = []
    for _ in range(n_runs):
        result = transcribe_file(audio_path, model_name=model_name)
        latencies.append(result["processing_time"])
        if result["real_time_factor"] is not None:
            rtf_values.append(result["real_time_factor"])

    duration = result["audio_duration"]
    avg_latency = float(np.mean(latencies))
    avg_rtf = float(np.mean(rtf_values)) if rtf_values else None

    return {
        "audio_duration": duration,
        "n_runs": n_runs,
        "avg_latency_s": round(avg_latency, 4),
        "avg_rtf": round(avg_rtf, 4) if avg_rtf is not None else None,
        "throughput": round(duration / avg_latency, 2) if avg_latency > 0 else None,
        "min_latency_s": round(float(np.min(latencies)), 4),
        "max_latency_s": round(float(np.max(latencies)), 4),
    }


# ── Batch transcription ──────────────────────────────────────────────────────


def run_batch(audio_dir: str, model_name: str) -> list[dict]:
    wav_files = sorted(f for f in os.listdir(audio_dir) if f.lower().endswith(".wav"))
    if not wav_files:
        print(f"No .wav files found in {audio_dir}")
        return []

    print(f"Found {len(wav_files)} .wav files.\n")
    results = []
    for wav in wav_files:
        path = os.path.join(audio_dir, wav)
        result = transcribe_file(path, model_name=model_name)
        print(f"  {wav}: {result['text']}  (RTF={result['real_time_factor']})")
        results.append({"file": wav, **result})
    return results


# ── Reporting ────────────────────────────────────────────────────────────────


def print_accuracy_report(results: list[dict], model_name: str) -> None:
    passed = sum(1 for r in results if r["passed"])
    total = len(results)
    avg_wer = float(np.mean([r["wer"] for r in results]))
    avg_acc = float(np.mean([r["accuracy"] for r in results]))

    print(f"\n{'='*70}")
    print(f" STT ACCURACY REPORT — model: {model_name} ".center(70))
    print(f"{'='*70}")

    for r in results:
        mark = "PASS" if r["passed"] else "FAIL"
        print(
            f"  [{mark}] {r['name']:<24} "
            f"WER={r['wer']:.2%}  "
            f"expected={r['expected']!r}  "
            f"got={r['actual']!r}"
        )

    print(f"\n{'─'*70}")
    print(f"  Total: {total}  |  Passed: {passed}  |  Failed: {total - passed}")
    print(f"  Avg WER: {avg_wer:.2%}  |  Avg Accuracy: {avg_acc:.2%}")
    print(f"{'='*70}\n")


def print_latency_report(latency: dict, model_name: str) -> None:
    print(f"\n{'='*70}")
    print(f" STT LATENCY REPORT — model: {model_name} ".center(70))
    print(f"{'='*70}")
    print(f"  Audio duration:  {latency['audio_duration']}s")
    print(f"  Runs:            {latency['n_runs']}")
    print(f"  Avg latency:     {latency['avg_latency_s']}s")
    print(
        f"  Min / Max:       {latency['min_latency_s']}s / {latency['max_latency_s']}s"
    )
    print(f"  Avg RTF:         {latency['avg_rtf']}x")
    print(f"  Throughput:      {latency['throughput']}x realtime")
    print(f"{'='*70}\n")


def save_accuracy_csv(results: list[dict], model_name: str) -> str:
    os.makedirs(RESULTS_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(RESULTS_DIR, f"accuracy_{model_name}_{ts}.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["name", "expected", "actual", "wer", "accuracy", "rtf", "passed"]
        )
        for r in results:
            writer.writerow(
                [
                    r["name"],
                    r["expected"],
                    r["actual"],
                    r["wer"],
                    r["accuracy"],
                    r.get("rtf", ""),
                    r["passed"],
                ]
            )
    return path


def save_latency_csv(latency: dict, model_name: str) -> str:
    os.makedirs(RESULTS_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(RESULTS_DIR, f"latency_{model_name}_{ts}.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "model",
                "audio_duration",
                "n_runs",
                "avg_latency_s",
                "avg_rtf",
                "throughput",
            ]
        )
        writer.writerow(
            [
                model_name,
                latency["audio_duration"],
                latency["n_runs"],
                latency["avg_latency_s"],
                latency["avg_rtf"],
                latency["throughput"],
            ]
        )
    return path


# ── CLI ──────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="STT benchmark — accuracy (WER) and latency (RTF)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--accuracy",
        action="store_true",
        help="Run accuracy benchmark against test_cases.json",
    )
    group.add_argument(
        "--latency",
        action="store_true",
        help="Run latency benchmark on a single audio file",
    )
    group.add_argument(
        "--batch", metavar="DIR", help="Transcribe all .wav files in a directory"
    )

    parser.add_argument(
        "--model",
        default="distil-large-v3",
        help="Whisper model (default: distil-large-v3)",
    )
    parser.add_argument("--language", default="en", help="Language code (default: en)")
    parser.add_argument(
        "--runs", type=int, default=3, help="Number of latency runs (default: 3)"
    )
    parser.add_argument("--audio", help="Audio file for latency benchmark")
    parser.add_argument(
        "--test-cases", default=TEST_CASES_FILE, help="Path to test_cases.json"
    )
    parser.add_argument("--no-vad", action="store_true", help="Disable VAD filter")
    parser.add_argument("--hotwords", default="", help="Hotwords hint for the model")
    parser.add_argument(
        "--initial-prompt", default="", help="Initial prompt for the model"
    )
    parser.add_argument(
        "--no-save", action="store_true", help="Skip saving results to CSV"
    )

    args = parser.parse_args()

    if args.accuracy:
        test_cases = load_test_cases(args.test_cases)
        print(
            f"Running accuracy benchmark: {len(test_cases)} test cases, model={args.model}"
        )
        results = run_accuracy(args.model, test_cases)
        print_accuracy_report(results, args.model)
        if not args.no_save:
            csv_path = save_accuracy_csv(results, args.model)
            print(f"Results saved to {csv_path}")

    elif args.latency:
        audio = args.audio
        if not audio:
            audio = os.path.join(RECORDINGS_DIR, "hello_frida.wav")
        if not os.path.isfile(audio):
            print(f"Error: file not found: {audio}")
            sys.exit(1)
        print(f"Running latency benchmark: {args.runs} runs, model={args.model}")
        latency = run_latency(audio, args.model, n_runs=args.runs)
        print_latency_report(latency, args.model)
        if not args.no_save:
            csv_path = save_latency_csv(latency, args.model)
            print(f"Results saved to {csv_path}")

    elif args.batch:
        if not os.path.isdir(args.batch):
            print(f"Error: directory not found: {args.batch}")
            sys.exit(1)
        run_batch(args.batch, args.model)


if __name__ == "__main__":
    main()
