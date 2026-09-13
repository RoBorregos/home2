#!/usr/bin/env python3
"""
Draft a task brief from a rulebook section, using an LLM as a compiler.

This is the only place an LLM touches task planning, and it runs offline, months before
the competition, with a human reviewing the output. At runtime the robot executes the
committed YAML; it does not ask a model what to do next.

The trick that makes this safe is that the brief parser is a strict verifier: unknown
skills, bad arity and malformed objectives are rejected, and the error is fed back to
the model for another attempt. A hallucinated `fold_laundry()` never reaches the robot.

Usage:
    python3 compile_brief.py --input rulebook_5_4.txt --task doing_laundry
    python3 compile_brief.py --input section.txt --task finals --show-prompt
    python3 compile_brief.py --input section.txt --task hric --out briefs/hric.yaml
"""

import argparse
import json
import os
import sys
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from task_manager.planner.brief import BriefError, parse_brief  # noqa: E402
from task_manager.skills.registry import REGISTRY, describe  # noqa: E402

DEFAULT_BASE_URL = os.environ.get("FRIDA_LLM_BASE_URL", "http://localhost:11434/v1")
DEFAULT_MODEL = os.environ.get("FRIDA_LLM_MODEL", "qwen3")
MAX_ATTEMPTS = 3

SYSTEM_PROMPT = """\
You convert RoboCup@Home rulebook sections into task briefs for a service robot.

A brief is YAML with this shape:

task: <snake_case name>
budget_s: <the test's time limit in seconds>
risk_posture: safe | aggressive
on_start: [<steps run once at the start>]
on_deadline: [<steps run when time runs out; always leave the arm safe>]
objectives:
  - id: <snake_case>
    points: <points from the scoresheet>
    template: [<steps>]
    requires: [<preconditions>]        # optional
    repeat_for: table_objects          # optional, repeats over detected objects
    once: true                         # optional
    at: <location>                     # optional, used for travel cost
    category: <kind of problem>        # optional
    penalty_risk: <negative number>    # optional, if it can score below zero
    fallback: <objective id>           # optional, a cheaper way to the same state

Rules you must follow:
1. Steps are calls to the skills listed below and NOTHING else. Inventing a skill is
   the most common mistake; if no skill fits, use the closest one and add a YAML
   comment saying a new primitive is needed.
2. Points must come from the scoresheet in the section. Do not guess.
3. Keep templates short. Probabilities multiply, so a six-step objective rarely
   completes; three or four steps is usually right.
4. Mark anything the scoresheet can score negative with penalty_risk.
5. Where the scoresheet prices human assistance at -0, add a fallback objective for it.
6. Output ONLY YAML. No prose, no code fences.

Available preconditions: arm_free, holding, objects_known, person_found.
"""


def skill_catalogue() -> str:
    """Every registered skill with its signature, so the model cannot invent one."""
    lines = []
    for name in sorted(REGISTRY):
        info = describe(name)
        args = ", ".join(
            arg["name"] if arg["required"] else f"{arg['name']}=..." for arg in info["args"]
        )
        lines.append(f"  {name}({args}) - {info['summary']}")
    return "\n".join(lines)


def build_prompt(section: str, task: str, previous_error: str = "") -> list:
    user = [
        f"Task name: {task}",
        "",
        "Available skills:",
        skill_catalogue(),
        "",
        "Rulebook section and scoresheet:",
        section.strip(),
    ]
    if previous_error:
        user += [
            "",
            "Your previous answer was rejected by the brief validator with this error:",
            previous_error,
            "Fix exactly that problem and output the corrected YAML.",
        ]
    return [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": "\n".join(user)},
    ]


def call_llm(messages: list, base_url: str, model: str, timeout: float = 180.0) -> str:
    """Plain OpenAI-compatible chat completion, so Ollama works with no extra deps."""
    payload = json.dumps(
        {"model": model, "messages": messages, "temperature": 0.2, "stream": False}
    ).encode()
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}/chat/completions",
        data=payload,
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {os.environ.get('OPENAI_API_KEY', 'ollama')}",
        },
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = json.loads(response.read())
    return body["choices"][0]["message"]["content"]


def strip_fences(text: str) -> str:
    """Models add code fences even when told not to."""
    cleaned = text.strip()
    if cleaned.startswith("```"):
        lines = cleaned.splitlines()
        lines = lines[1:]
        if lines and lines[-1].strip().startswith("```"):
            lines = lines[:-1]
        cleaned = "\n".join(lines)
    # drop a reasoning preamble some local models emit
    if "<think>" in cleaned and "</think>" in cleaned:
        cleaned = cleaned.split("</think>", 1)[1].strip()
    return cleaned


def compile_brief(section: str, task: str, complete, attempts: int = MAX_ATTEMPTS) -> tuple:
    """
    Draft, validate, and retry with the validator's complaint fed back.

    `complete(messages) -> str` is injected so this is testable without an LLM.
    Returns (yaml_text, brief). Raises BriefError if every attempt fails.
    """
    import yaml

    error = ""
    last_text = ""
    for attempt in range(1, attempts + 1):
        raw = complete(build_prompt(section, task, error))
        last_text = strip_fences(raw)
        try:
            data = yaml.safe_load(last_text)
            if not isinstance(data, dict):
                raise BriefError(f"expected a YAML mapping, got {type(data).__name__}")
            brief = parse_brief(data)
            return last_text, brief
        except (BriefError, yaml.YAMLError) as problem:
            error = str(problem)
            print(f"  attempt {attempt} rejected: {error}", file=sys.stderr)
    raise BriefError(f"no valid brief after {attempts} attempts. Last error: {error}\n{last_text}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="text file with the rulebook section")
    parser.add_argument("--task", required=True, help="task name for the brief")
    parser.add_argument("--out", default="", help="write the brief here instead of stdout")
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--show-prompt", action="store_true", help="print the prompt and exit")
    args = parser.parse_args()

    with open(args.input, encoding="utf-8") as handle:
        section = handle.read()

    if args.show_prompt:
        for message in build_prompt(section, args.task):
            print(f"--- {message['role']} ---\n{message['content']}\n")
        return 0

    def complete(messages):
        return call_llm(messages, args.base_url, args.model)

    try:
        text, brief = compile_brief(section, args.task, complete)
    except urllib.error.URLError as error:
        print(f"Could not reach {args.base_url}: {error}", file=sys.stderr)
        print("Start Ollama, or set FRIDA_LLM_BASE_URL.", file=sys.stderr)
        return 2
    except BriefError as error:
        print(f"Compilation failed: {error}", file=sys.stderr)
        return 1

    print(
        f"Drafted '{brief.task}': {len(brief.objectives)} objectives, "
        f"{len(brief.triggers)} triggers, budget {brief.budget_s:.0f}s",
        file=sys.stderr,
    )
    print(
        "REVIEW THIS BEFORE COMMITTING — point values decide what the robot chases.",
        file=sys.stderr,
    )

    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
        print(f"Wrote {args.out}", file=sys.stderr)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
