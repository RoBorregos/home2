#!/usr/bin/env python3
"""Cold time-to-first-token of rbrgs for every GenerateCommandList test, with the prompt the BAML tests send.

Reads robot_commands.baml + the test .baml files concatenated on stdin; prints model, run, test, ttft_ms, prompt_tokens, prefill_ms as TSV.
"""

import argparse
import json
import re
import sys
import time
import urllib.request


def parse(baml):
    fn = re.search(r"function GenerateCommandList\b.*?\n}", baml, re.S).group(0)
    system = re.search(
        r'_\.role\("system"\) }}(.*?){{ _\.role\("user"\)', fn, re.S
    ).group(1)
    system = " ".join(line.strip() for line in system.strip().splitlines())
    tests = re.findall(
        r"^test (\w+) {\s*functions \[GenerateCommandList\]\s*args {\s*request \"([^\"]*)\"",
        baml,
        re.M,
    )
    return system, tests


def first_token(port, system, request):
    body = json.dumps(
        {
            "model": "rbrgs",
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": request},
            ],
            "temperature": 0,
            "max_tokens": 1,
            "stream": True,
            "cache_prompt": False,
            "timings_per_token": True,
        }
    ).encode()
    req = urllib.request.Request(
        f"http://localhost:{port}/v1/chat/completions",
        body,
        {"Content-Type": "application/json"},
    )
    start = time.perf_counter()
    ttft, timings = None, {}
    with urllib.request.urlopen(req, timeout=300) as resp:
        for raw in resp:
            line = raw.decode().strip()
            if not line.startswith("data: ") or line == "data: [DONE]":
                continue
            chunk = json.loads(line[6:])
            delta = (chunk.get("choices") or [{}])[0].get("delta", {})
            if ttft is None and delta.get("content"):
                ttft = (time.perf_counter() - start) * 1000
            timings = chunk.get("timings", timings)
    return ttft, timings.get("prompt_n"), timings.get("prompt_ms")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--run", default="1")
    ap.add_argument("--port", type=int, default=11435)
    args = ap.parse_args()

    system, tests = parse(sys.stdin.read())
    for name, request in tests:
        ttft, prompt_n, prompt_ms = first_token(args.port, system, request)
        fmt = lambda v: "" if v is None else f"{v:.0f}"  # noqa: E731
        print(
            f"{args.model}\t{args.run}\t{name}\t{fmt(ttft)}\t{fmt(prompt_n)}\t{fmt(prompt_ms)}",
            flush=True,
        )


if __name__ == "__main__":
    main()
