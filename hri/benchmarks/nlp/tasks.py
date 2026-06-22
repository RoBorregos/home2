"""Perf side-channel: timed inferences against llama-server.

Uses stdlib urllib so the integration container needs no extra dependency.
Prompts are imported from the live NLP package (hri/packages/nlp/nlp/assets/dialogs.py)
so model behavior stays in lockstep with production. Accuracy scoring happens in
task_manager/scripts/test/test_hri_manager.py via the existing HRI pipeline.
"""

import json
import os
import sys
import time
import urllib.request
from typing import Optional

# The `nlp` Python package lives in hri/packages/nlp/nlp and is NOT colcon-built
# inside the integration container. Add its parent dir so we can import the
# canonical prompts (single source of truth shared with production).
_NLP_PKG_PARENT = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "packages", "nlp")
)
if _NLP_PKG_PARENT not in sys.path:
    sys.path.insert(0, _NLP_PKG_PARENT)

from nlp.assets.dialogs import (  # noqa: E402
    get_extract_data_args,
    get_is_answer_negative_args,
    get_is_answer_positive_args,
)


def _messages_only(dialog_args):
    return dialog_args[0]


def _stream_chat(url: str, model: str, messages: list, max_tokens: int):
    """Yield parsed SSE events from llama-server /v1/chat/completions."""
    payload = {
        "model": model,
        "messages": messages,
        "stream": True,
        "stream_options": {"include_usage": True},
        "max_tokens": max_tokens,
    }
    req = urllib.request.Request(
        url.rstrip("/") + "/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Authorization": "Bearer ollama",
            "Accept": "text/event-stream",
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        for raw in resp:
            line = raw.decode("utf-8", errors="replace").strip()
            if not line.startswith("data:"):
                continue
            data = line[len("data:") :].strip()
            if data == "[DONE]":
                return
            try:
                yield json.loads(data)
            except json.JSONDecodeError:
                continue


def _run_timed(url: str, model: str, messages: list, max_tokens: int = 128) -> dict:
    t_start = time.perf_counter()
    t_first: Optional[float] = None
    t_end: Optional[float] = None
    completion_tokens = 0

    for chunk in _stream_chat(url, model, messages, max_tokens):
        now = time.perf_counter()
        choices = chunk.get("choices") or []
        if choices:
            delta = choices[0].get("delta", {}) or {}
            text = (delta.get("content") or "") + (delta.get("reasoning_content") or "")
            if text and t_first is None:
                t_first = now
            if choices[0].get("finish_reason") in ("stop", "length"):
                t_end = now
        usage = chunk.get("usage")
        if usage:
            completion_tokens = usage.get("completion_tokens") or completion_tokens

    if t_first is None or t_end is None:
        return {
            "ttft_ms": None,
            "total_ms": None,
            "tokens_per_s": None,
            "decode_tokens_per_s": None,
        }

    total_s = t_end - t_start
    gen_s = t_end - t_first
    return {
        "ttft_ms": (t_first - t_start) * 1000,
        "total_ms": total_s * 1000,
        "tokens_per_s": (completion_tokens / total_s)
        if total_s > 0 and completion_tokens > 0
        else 0.0,
        "decode_tokens_per_s": (completion_tokens / gen_s)
        if gen_s > 0 and completion_tokens > 0
        else 0.0,
    }


class ExtractDataTask:
    name = "extract_data"

    @staticmethod
    def perf_messages() -> list:
        return _messages_only(
            get_extract_data_args(
                "My name is Carlos and I would like a glass of water.", "drink"
            )
        )


class IsPositiveTask:
    name = "is_positive"

    @staticmethod
    def perf_messages() -> list:
        return _messages_only(get_is_answer_positive_args("Yes, that's correct"))


class IsNegativeTask:
    name = "is_negative"

    @staticmethod
    def perf_messages() -> list:
        return _messages_only(get_is_answer_negative_args("No, that's wrong"))


def run_perf(url: str, model: str, task_cls, runs: int) -> dict:
    msgs = task_cls.perf_messages()
    ttft_list, tps_list, decode_tps_list = [], [], []
    for _ in range(runs):
        r = _run_timed(url, model, msgs)
        if r["ttft_ms"] is not None:
            ttft_list.append(r["ttft_ms"])
            tps_list.append(r["tokens_per_s"])
            decode_tps_list.append(r["decode_tokens_per_s"])
    if not ttft_list:
        return {
            "avg_ttft_ms": None,
            "avg_tokens_per_s": None,
            "avg_decode_tokens_per_s": None,
        }
    return {
        "avg_ttft_ms": round(sum(ttft_list) / len(ttft_list), 1),
        "avg_tokens_per_s": round(sum(tps_list) / len(tps_list), 1),
        "avg_decode_tokens_per_s": round(
            sum(decode_tps_list) / len(decode_tps_list), 1
        ),
    }


TASK_REGISTRY = {
    "extract_data": ExtractDataTask,
    "is_positive": IsPositiveTask,
    "is_negative": IsNegativeTask,
}
