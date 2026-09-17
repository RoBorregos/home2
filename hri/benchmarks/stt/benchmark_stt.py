#!/usr/bin/env python3
"""
STT benchmark library — accuracy (WER) and latency (RTF) for faster-whisper models.

Reporting is handled by report.py. This module provides the core functions:
  run_accuracy()  — run test cases, return per-case WER / pass-fail
  run_latency()   — measure latency and RTF on a single audio file
  transcribe_file() — transcribe a single audio file
"""

import json
import os
import re
import struct
import tempfile
import time
import wave

import numpy as np
from faster_whisper import WhisperModel

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RECORDINGS_DIR = os.path.join(SCRIPT_DIR, "recordings")
TEST_CASES_FILE = os.path.join(SCRIPT_DIR, "test_cases.json")

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


def _apply_gain(audio_path: str, gain: float) -> str:
    """Apply linear gain to a WAV file. Returns path to a temporary copy."""
    with wave.open(audio_path, "rb") as wf:
        params = wf.getparams()
        frames = wf.readframes(params.nframes)

    # Unpack all samples, scale, clamp
    fmt = "<{n}h".format(n=params.nframes * params.nchannels)
    samples = list(struct.unpack(fmt, frames))
    max_sample = 32767
    scaled = []
    for s in samples:
        v = int(s * gain)
        scaled.append(max(-max_sample, min(max_sample, v)))

    out = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    with wave.open(out.name, "wb") as wf_out:
        wf_out.setparams(params)
        wf_out.writeframes(struct.pack(fmt, *scaled))

    return out.name


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
        gain = tc.get("gain", 1.0)

        if not os.path.exists(audio_path):
            results.append(
                {
                    "name": tc["name"],
                    "expected": expected,
                    "actual": "FILE_NOT_FOUND",
                    "wer": 1.0,
                    "passed": False,
                }
            )
            continue

        temp_file = None
        if gain != 1.0:
            temp_file = _apply_gain(audio_path, gain)
            audio_path = temp_file

        try:
            result = transcribe_file(audio_path, model_name=model_name)
        finally:
            if temp_file:
                os.unlink(temp_file)

        actual = result["text"]
        wer = min(calculate_wer(expected, actual), 1.0)
        passed = wer == 0.0

        results.append(
            {
                "name": tc["name"],
                "expected": expected,
                "actual": actual,
                "wer": round(wer, 4),
                "rtf": result["real_time_factor"],
                "processing_time": result["processing_time"],
                "audio_duration": result["audio_duration"],
                "passed": passed,
            }
        )
    return results


def run_latency(
    audio_path: str,
    model_name: str,
    n_runs: int = 3,
) -> dict:
    if n_runs < 1:
        raise ValueError(f"n_runs must be >= 1, got {n_runs}")

    # Warmup run — first transcribe is slower (CUDA kernel init, etc.)
    transcribe_file(audio_path, model_name=model_name)

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
