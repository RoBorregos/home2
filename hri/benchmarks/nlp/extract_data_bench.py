#!/usr/bin/env python3
"""Compare spaCy vs LLM vs the /extract_data pipeline without ROS.

Runs inside hri-ros (needs the LLM server on localhost:11434):
    python3 /workspace/src/hri/benchmarks/nlp/extract_data_bench.py            # batch
    python3 /workspace/src/hri/benchmarks/nlp/extract_data_bench.py --live     # REPL

Datasets use the nlp/test format: [full_text, query, context, expected].
"""

import argparse
import json
import os
import re
import string
import sys
import time

import spacy
from nlp.assets.data_extraction_priority import (
    LOC_PRIORITY_LABELS,
    NAME_PRIORITY_LABELS,
    extract_by_priority,
)
from nlp.assets.dialogs import NO_THINKING, get_extract_data_args, strip_thinking
from openai import OpenAI

from frida_constants.hri_constants import MODEL

BENCH_DIR = os.path.dirname(os.path.abspath(__file__))
NLP_DIR = os.path.normpath(os.path.join(BENCH_DIR, "..", "..", "packages", "nlp"))
SPACY_DIR = os.path.join(NLP_DIR, "assets", "extract_data")
DEFAULT_DATASETS = [
    os.path.join(NLP_DIR, "test", "data_extractor.json"),
    os.path.join(BENCH_DIR, "extract_data_hric.json"),
]
SPACY_LABELS = {
    "name": NAME_PRIORITY_LABELS,
    "loc": LOC_PRIORITY_LABELS,
    "location": LOC_PRIORITY_LABELS,
}


def normalize(text: str) -> str:
    text = text.lower().translate(str.maketrans("", "", string.punctuation))
    return " ".join(text.split())


class Extractor:
    def __init__(self, base_url: str, temperature: float, spacy_model: str):
        path = os.path.join(SPACY_DIR, spacy_model)
        self.nlp = spacy.load(path if os.path.isdir(path) else spacy_model)
        self.client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY", "ollama"), base_url=base_url
        )
        self.temperature = temperature

    def spacy(self, text: str, query: str):
        labels = SPACY_LABELS.get(query)
        doc = self.nlp(text)
        ents = [(e.text, e.label_) for e in doc.ents]
        return (extract_by_priority(doc.ents, labels) if labels else None), ents

    def llm(self, text: str, query: str, context: str) -> str:
        if query.startswith("LLM_"):
            query = query[4:]
        messages, response_format = get_extract_data_args(text, query, context)
        try:
            content = (
                self.client.beta.chat.completions.parse(
                    model=MODEL.EXTRACT_INFO_REQUESTED.value,
                    temperature=self.temperature,
                    messages=messages,
                    response_format=response_format,
                    extra_body=NO_THINKING,
                )
                .choices[0]
                .message.content
            )
            return json.loads(strip_thinking(content)).get("data") or ""
        except Exception as e:
            print(f"  LLM error: {e}", file=sys.stderr)
            return ""

    def run(self, text: str, query: str, context: str) -> dict:
        """Same decisions as scripts/extract_data.py, with per-stage results and timings."""
        out = {"spacy": None, "ents": [], "llm": None, "t_spacy": 0.0, "t_llm": 0.0}
        t = time.perf_counter()
        out["spacy"], out["ents"] = self.spacy(text, query)
        out["t_spacy"] = time.perf_counter() - t
        t = time.perf_counter()
        out["llm"] = self.llm(text, query, context)
        out["t_llm"] = time.perf_counter() - t

        if len(text.strip().split(" ")) < 2:
            out["pipeline"], out["path"] = text, "passthrough"
        elif out["spacy"]:
            out["pipeline"], out["path"] = out["spacy"], "spacy"
        else:
            out["pipeline"], out["path"] = out["llm"], "llm"
        return out


def mark(value, expected) -> str:
    if value is None:
        return "-"
    return "OK" if normalize(value) == normalize(expected) else "XX"


def cell(value, expected) -> str:
    return f"{mark(value, expected)} {(value if value is not None else '')[:16]}"


def batch(
    ex: Extractor, datasets: list[str], query_filter: str | None, verbose: bool
) -> None:
    cases = []
    for path in datasets:
        with open(path) as f:
            cases += [(os.path.basename(path), *c) for c in json.load(f)]
    if query_filter:
        cases = [c for c in cases if re.search(query_filter, c[2])]

    stats = {k: {"ok": 0, "n": 0, "t": 0.0} for k in ("spacy", "llm", "pipeline")}
    failures = []
    print(
        f"{'query':<14} {'expected':<16} {'spacy':<20} {'llm':<20} {'pipeline':<20} {'t_llm':>6}"
    )
    for src, text, query, context, expected in cases:
        r = ex.run(text, query, context)
        for k in stats:
            if r[k] is None:
                continue
            stats[k]["n"] += 1
            stats[k]["ok"] += normalize(r[k]) == normalize(expected)
        stats["spacy"]["t"] += r["t_spacy"]
        stats["llm"]["t"] += r["t_llm"]
        print(
            f"{query[:14]:<14} {expected[:16]:<16} {cell(r['spacy'], expected):<20} {cell(r['llm'], expected):<20} "
            f"{cell(r['pipeline'], expected):<20} {r['t_llm']:>5.2f}s"
        )
        if verbose:
            print(f"    text: {text!r}  ents: {r['ents']}  path: {r['path']}")
        if normalize(r["pipeline"]) != normalize(expected):
            failures.append((src, text, query, expected, r))

    print("\n=== Summary ===")
    for k, s in stats.items():
        if s["n"]:
            print(f"{k:<9} {s['ok']}/{s['n']} ({100 * s['ok'] / s['n']:.0f}%)")
    n = len(cases) or 1
    print(
        f"avg time  spacy {1000 * stats['spacy']['t'] / n:.0f} ms | llm {1000 * stats['llm']['t'] / n:.0f} ms"
    )

    if failures:
        print("\n=== Pipeline failures ===")
        for src, text, query, expected, r in failures:
            print(
                f"[{src}] {query}: {text!r}\n    expected={expected!r} got={r['pipeline']!r} "
                f"via {r['path']} | spacy={r['spacy']!r} llm={r['llm']!r} ents={r['ents']}"
            )


def live(ex: Extractor, query: str, context: str) -> None:
    print(
        "Type a sentence. Commands: :q <query> (name, location, LLM_drink...), :c <context>, :exit"
    )
    while True:
        try:
            line = input(f"[{query}]> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue
        if line == ":exit":
            return
        if line.startswith(":q "):
            query = line[3:].strip()
            continue
        if line.startswith(":c "):
            context = line[3:].strip()
            continue
        r = ex.run(line, query, context)
        print(f"  ents     : {r['ents']}")
        if r["spacy"] is not None:
            print(f"  spacy    : {r['spacy']!r}  ({1000 * r['t_spacy']:.0f} ms)")
        print(f"  llm      : {r['llm']!r}  ({r['t_llm']:.2f} s)")
        print(f"  pipeline : {r['pipeline']!r}  via {r['path']}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--live", action="store_true", help="interactive mode")
    parser.add_argument("--query", default="name", help="initial query in live mode")
    parser.add_argument("--context", default="", help="initial context in live mode")
    parser.add_argument(
        "--data", nargs="+", default=DEFAULT_DATASETS, help="dataset json files"
    )
    parser.add_argument("--filter", help="regex on query, e.g. '^name$' or drink")
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="print text, entities and path per case",
    )
    parser.add_argument("--base-url", default="http://localhost:11434/v1")
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--spacy-model", default="en_core_web_md")
    args = parser.parse_args()

    ex = Extractor(args.base_url, args.temperature, args.spacy_model)
    if args.live:
        live(ex, args.query, args.context)
    else:
        batch(ex, args.data, args.filter, args.verbose)


if __name__ == "__main__":
    main()
