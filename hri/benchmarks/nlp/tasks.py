"""Perf and JSON-conformance side-channel against an OpenAI-compatible server.

Prompts and response schemas come from nlp.assets.dialogs so the side-channel
exercises exactly what production sends.
"""

import json
import os
import re
import statistics
import sys
import time
import urllib.error
import urllib.request
from typing import Optional

# nlp is a plain Python package (not colcon-built in the integration container),
# so make its parent dir importable before pulling the canonical prompts.
_NLP_PKG_PARENT = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "packages", "nlp")
)
if _NLP_PKG_PARENT not in sys.path:
    sys.path.insert(0, _NLP_PKG_PARENT)

from nlp.assets.dialogs import (  # noqa: E402
    NO_THINKING,
    get_extract_data_args,
    get_is_answer_negative_args,
    get_is_answer_positive_args,
    get_is_coherent_dialog,
    get_previous_command_answer,
)

_THINK_RE = re.compile(r"<think>.*?</think>", re.DOTALL)
_JSON_SPAN_RE = re.compile(r"\{.*\}", re.DOTALL)

WARMUP_RUNS = 1


def _normalize_dialog(dialog):
    """dialogs.py returns a (messages, schema) tuple, a dict, or a bare list."""
    if isinstance(dialog, dict):
        return dialog["messages"], dialog.get("response_format")
    if isinstance(dialog, tuple):
        return dialog[0], dialog[1] if len(dialog) > 1 else None
    return dialog, None


def _schema_request(schema_model):
    if schema_model is None:
        return None
    return {
        "type": "json_schema",
        "json_schema": {
            "name": schema_model.__name__,
            "schema": schema_model.model_json_schema(),
        },
    }


def _validates(text: str, schema_model) -> bool:
    """True when the reply parses as JSON and satisfies the production schema."""
    body = _THINK_RE.sub("", text).strip()
    if body.startswith("```"):
        body = body.strip("`")
        body = body.split("\n", 1)[-1] if "\n" in body else body
    try:
        parsed = json.loads(body)
    except json.JSONDecodeError:
        span = _JSON_SPAN_RE.search(body)
        if not span:
            return False
        try:
            parsed = json.loads(span.group(0))
        except json.JSONDecodeError:
            return False
    try:
        schema_model(**parsed)
    except Exception:
        return False
    return True


def _post_stream(url: str, payload: dict, timeout: int = 60):
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
    resp = urllib.request.urlopen(req, timeout=timeout)
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


def _run_timed(
    url: str, model: str, messages: list, schema_model, max_tokens: int = 128
) -> dict:
    """One timed inference. Never invents a token count: missing usage -> None."""
    payload = {
        "model": model,
        "messages": messages,
        "stream": True,
        "stream_options": {"include_usage": True},
        "max_tokens": max_tokens,
        "temperature": 0,
        **NO_THINKING,
    }
    schema_req = _schema_request(schema_model)
    schema_mode = "none"
    if schema_req:
        payload["response_format"] = schema_req
        schema_mode = "json_schema"

    result = {
        "ttft_ms": None,
        "total_ms": None,
        "tokens_per_s": None,
        "decode_tokens_per_s": None,
        "completion_tokens": None,
        "usage_missing": False,
        "json_ok": None,
        "schema_mode": schema_mode,
        "error": None,
    }

    for attempt in ("primary", "json_object_fallback"):
        t_start = time.perf_counter()
        t_first: Optional[float] = None
        t_end: Optional[float] = None
        completion_tokens = None
        chunks = []
        try:
            for chunk in _post_stream(url, payload):
                now = time.perf_counter()
                choices = chunk.get("choices") or []
                if choices:
                    delta = choices[0].get("delta", {}) or {}
                    text = (delta.get("content") or "") + (
                        delta.get("reasoning_content") or ""
                    )
                    if text:
                        chunks.append(delta.get("content") or "")
                        if t_first is None:
                            t_first = now
                    if choices[0].get("finish_reason") in ("stop", "length"):
                        t_end = now
                usage = chunk.get("usage")
                if usage and usage.get("completion_tokens"):
                    completion_tokens = usage["completion_tokens"]
        except urllib.error.HTTPError as e:
            # A backend that rejects the production schema is a real finding:
            # retry once unconstrained and record the degraded mode.
            if attempt == "primary" and schema_req and e.code in (400, 404, 422):
                payload["response_format"] = {"type": "json_object"}
                result["schema_mode"] = "json_object"
                continue
            result["error"] = f"HTTP {e.code}: {e.reason}"
            return result
        except Exception as e:
            result["error"] = f"{type(e).__name__}: {e}"
            return result
        break

    if t_first is None or t_end is None:
        result["error"] = "no content in stream"
        return result

    total_s = t_end - t_start
    gen_s = t_end - t_first
    result["ttft_ms"] = (t_first - t_start) * 1000
    result["total_ms"] = total_s * 1000
    result["completion_tokens"] = completion_tokens

    if completion_tokens is None:
        result["usage_missing"] = True
    else:
        if total_s > 0:
            result["tokens_per_s"] = completion_tokens / total_s
        if gen_s > 0:
            result["decode_tokens_per_s"] = completion_tokens / gen_s

    if schema_model is not None:
        result["json_ok"] = _validates("".join(chunks), schema_model)

    return result


class ExtractDataTask:
    name = "extract_data"

    @staticmethod
    def perf_dialog():
        return get_extract_data_args(
            "My name is Carlos and I would like a glass of water.", "drink"
        )


class IsPositiveTask:
    name = "is_positive"

    @staticmethod
    def perf_dialog():
        return get_is_answer_positive_args("Yes, that's correct")


class IsNegativeTask:
    name = "is_negative"

    @staticmethod
    def perf_dialog():
        return get_is_answer_negative_args("No, that's wrong")


class IsCoherentTask:
    name = "is_coherent"

    @staticmethod
    def perf_dialog():
        return get_is_coherent_dialog("Go to the kitchen and pick up the apple")


class LLMWrapperTask:
    name = "llm_wrapper"

    @staticmethod
    def perf_dialog():
        return get_previous_command_answer(
            "The robot picked up a red apple from the kitchen table.",
            "What object did the robot pick up?",
        )


def _pct(values: list, q: float) -> float:
    """Nearest-rank percentile; stable for the small n a benchmark run gives."""
    ordered = sorted(values)
    idx = max(0, min(len(ordered) - 1, int(round(q * (len(ordered) - 1)))))
    return ordered[idx]


def _summarize(values: list, prefix: str) -> dict:
    if not values:
        return {f"avg_{prefix}": None, f"p50_{prefix}": None, f"p95_{prefix}": None}
    return {
        f"avg_{prefix}": round(statistics.mean(values), 1),
        f"p50_{prefix}": round(_pct(values, 0.50), 1),
        f"p95_{prefix}": round(_pct(values, 0.95), 1),
    }


def run_perf(
    url: str, model: str, task_cls, runs: int, warmup: int = WARMUP_RUNS
) -> dict:
    """Timed runs after `warmup` discarded ones, so run 1 cold start is excluded."""
    messages, schema_model = _normalize_dialog(task_cls.perf_dialog())

    for _ in range(max(0, warmup)):
        _run_timed(url, model, messages, schema_model)

    ttft, tps, decode_tps = [], [], []
    json_ok = json_total = 0
    usage_missing = False
    errors = []
    schema_modes = set()

    for _ in range(runs):
        r = _run_timed(url, model, messages, schema_model)
        schema_modes.add(r["schema_mode"])
        if r["error"]:
            errors.append(r["error"])
            continue
        ttft.append(r["ttft_ms"])
        if r["usage_missing"]:
            usage_missing = True
        else:
            tps.append(r["tokens_per_s"])
            decode_tps.append(r["decode_tokens_per_s"])
        if r["json_ok"] is not None:
            json_total += 1
            json_ok += 1 if r["json_ok"] else 0

    out = {}
    out.update(_summarize(ttft, "ttft_ms"))
    out.update(_summarize(tps, "tokens_per_s"))
    out.update(_summarize(decode_tps, "decode_tokens_per_s"))
    out["runs_ok"] = len(ttft)
    out["runs_requested"] = runs
    out["usage_missing"] = usage_missing
    out["schema_mode"] = "/".join(sorted(schema_modes)) if schema_modes else "none"
    out["json_checked"] = json_total
    out["json_ok"] = json_ok
    out["json_fail_rate"] = (
        round((json_total - json_ok) / json_total, 3) if json_total else None
    )
    if errors:
        out["errors"] = errors[:5]
    return out


def probe_backend(url: str) -> dict:
    """Config stamp: what actually served the run."""
    info = {"url": url, "backend": "unknown"}
    root = url.rstrip("/")
    if root.endswith("/v1"):
        root = root[: -len("/v1")]

    try:
        with urllib.request.urlopen(root + "/props", timeout=5) as resp:
            props = json.loads(resp.read().decode("utf-8"))
        info["backend"] = "llama.cpp"
        info["model_path"] = props.get("model_path")
        info["n_ctx"] = props.get("n_ctx")
        gen = props.get("default_generation_settings") or {}
        for key in ("n_ctx", "n_predict", "temperature", "top_p", "samplers"):
            if key in gen:
                info.setdefault("generation", {})[key] = gen[key]
        if "build_info" in props:
            info["build_info"] = props["build_info"]
        return info
    except Exception:
        pass

    try:
        with urllib.request.urlopen(root + "/api/version", timeout=5) as resp:
            info["backend"] = "ollama"
            info["version"] = json.loads(resp.read().decode("utf-8")).get("version")
    except Exception:
        pass

    try:
        with urllib.request.urlopen(root + "/v1/models", timeout=5) as resp:
            data = json.loads(resp.read().decode("utf-8")).get("data") or []
            info["models"] = [m.get("id") for m in data]
    except Exception:
        pass

    return info


TASK_REGISTRY = {
    "extract_data": ExtractDataTask,
    "is_positive": IsPositiveTask,
    "is_negative": IsNegativeTask,
    "is_coherent": IsCoherentTask,
    "llm_wrapper": LLMWrapperTask,
}
