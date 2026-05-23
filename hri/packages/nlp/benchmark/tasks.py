"""Benchmark task definitions: prompt builders, API calls, and scoring."""

import json
import re
import time
from typing import Any, Optional

from openai import OpenAI
from pydantic import BaseModel, Field


# ── Pydantic schemas (mirrors hri/packages/nlp/nlp/assets/schemas.py) ─────────


class ExtractedData(BaseModel):
    data: str
    rationale: str = Field(default="")


class IsAnswerPositive(BaseModel):
    is_positive: bool


class IsAnswerNegative(BaseModel):
    is_negative: bool


# ── Prompt builders (verbatim from hri/packages/nlp/nlp/assets/dialogs.py) ────


def _extract_data_messages(
    full_text: str, data_to_extract: str, context: str = ""
) -> list:
    user_content = f"<full_text>{full_text}</full_text>\n<extract_data>{data_to_extract}</extract_data>"
    if context:
        user_content += f"\n<explanation>{context}</explanation>"

    system = f"""You will receive a text (`full_text`) and a specific target (`extract_data`). (Optional) Additional explanation (`explanation`) to help clarify ambiguous cases. Your task is to extract and return the closest relevant word or phrase that directly answers the target.

### Extraction Rules:
- Return the MOST RELEVANT WORD OR PHRASE that best corresponds to `extract_data`, considering its contextual meaning within the sentence.
- Do NOT return the target word (`extract_data`) itself unless it is the best available answer.
- If multiple possible matches exist, return the MOST CONTEXTUALLY RELEVANT one (e.g., a noun or phrase describing the requested information).
- If no relevant match is found, return an empty string (`""`).
- If `full_text` is missing, empty, or consists of only a single word or short phrase that directly corresponds to `extract_data`, return `full_text` as the result.
- If present, use the `explanation` to help clarify ambiguous cases.

### Examples:


#### Example 1:
**INPUT:**
<full_text>
    There is a cat in the house.
</full_text>
<extract_data>
    food
</extract_data>

**OUTPUT:**
{ExtractedData(data="").model_dump_json()}

#### Example 2:
**INPUT:**
<full_text>
    The restaurant serves delicious Italian food.
</full_text>
<extract_data>
    food
</extract_data>

**OUTPUT:**
{ExtractedData(data="Italian food").model_dump_json()}

#### Example 3:
**INPUT:**
<full_text>
    My name is Juan and I like lemonade.
</full_text>
<extract_data>
    drink
</extract_data>

**OUTPUT:**
{ExtractedData(data="lemonade").model_dump_json()}

#### Example 4:
**INPUT:**
<full_text>
    Juan and I like to play basketball.
</full_text>
<extract_data>
    name
</extract_data>

**OUTPUT:**
{ExtractedData(data="Juan").model_dump_json()}

#### Example 5:
**INPUT:**
<full_text>
    Elis
</full_text>
<extract_data>
    name
</extract_data>

**OUTPUT:**
{ExtractedData(data="Elis").model_dump_json()}

Ensure that the extracted data is always **the most contextually relevant** answer, not simply the target term itself."""

    return [
        {"role": "system", "content": system},
        {"role": "user", "content": user_content + " /no_think"},
    ]


def _is_positive_messages(text: str) -> list:
    system = f"""You will be given a statement, and your task is to determine whether it is a **positive confirmation** or not. A **positive confirmation** is an explicit or implicit agreement, affirmation, or confirmation (e.g., 'yes', 'that's right', 'correct', 'absolutely'). A **negative response** includes disagreement, uncertainty, negation, or lack of understanding (e.g., 'no', 'I don't know', 'wrong', 'not sure'). If the statement is ambiguous or unclear, assume it is negative.

### Guidelines:
- Only return a boolean value: **true** for positive confirmation, **false** otherwise.
- Ignore irrelevant sentiment (e.g., 'I am happy' is **not** a confirmation, so return false).
- Consider implicit confirmations like 'exactly' or 'of course' as positive.
- Treat uncertain responses like 'maybe' or 'I guess' as negative.

### Examples:
- **Input:** 'Yes'
**Output:**
{IsAnswerPositive(is_positive=True).model_dump_json()}

**Input:** 'That's correct'
**Output:**
{IsAnswerPositive(is_positive=True).model_dump_json()}

**Input:** 'I agree'
**Output:**
{IsAnswerPositive(is_positive=True).model_dump_json()}

**Input:** 'Wrong'
**Output:**
{IsAnswerPositive(is_positive=False).model_dump_json()}

**Input:** 'I don't know'
**Output:**
{IsAnswerPositive(is_positive=False).model_dump_json()}

**Input:** 'Huh?'
**Output:**
{IsAnswerPositive(is_positive=False).model_dump_json()}

**Input:** 'Maybe'
**Output:**
{IsAnswerPositive(is_positive=False).model_dump_json()}
"""
    return [
        {"role": "system", "content": system},
        {"role": "user", "content": text + " /no_think"},
    ]


def _is_negative_messages(text: str) -> list:
    system = f"""You will receive a short statement. Your task is to determine if the statement is a **negative confirmation**.

A negative confirmation includes:
- explicit disagreement or rejection (e.g., "no", "that's wrong", "incorrect")

But does **not** include:
- expressions of uncertainty or lack of confidence (e.g., "maybe", "I'm not sure", "I don't think so", "I doubt it")
- explicit affirmation (e.g. "yes", "correct", "sure", "absolutely", "of course")

**Always respond with a single JSON object with one field `is_negative` (boolean).**

### Examples:
- **Input:** 'Yes'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

**Input:** 'That's not correct'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}

**Input:** 'I don't agree'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}

**Input:** 'Wrong'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}

**Input:** 'I don't know'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

**Input:** 'That's right'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

**Input:** 'Maybe'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}

**Input:** 'No'
**Output:**
{IsAnswerNegative(is_negative=True).model_dump_json()}

**Input:** 'Absolutely'
**Output:**
{IsAnswerNegative(is_negative=False).model_dump_json()}
"""
    return [
        {"role": "system", "content": system},
        {"role": "user", "content": text + " /no_think"},
    ]


def _command_interpreter_messages(command: str) -> list:
    return [
        {
            "role": "system",
            "content": (
                "You are a robot command parser. Given a natural language command, output a JSON object "
                "with a 'commands' array. Each element has: action (string), characteristic (string or null), "
                "complement (string or null). Common actions: go_to, pick_object, place_object, give_object, "
                "find_person, find_person_by_name, follow_person_until, guide_person_to, count, get_person_info, "
                "get_visual_info, say_with_context. Output ONLY valid JSON, no explanation."
            ),
        },
        {"role": "user", "content": command + " /no_think"},
    ]


# ── Shared timed inference helper ─────────────────────────────────────────────


def _run_timed(
    client: OpenAI, model: str, messages: list, max_tokens: int = 128
) -> dict:
    """Run a single inference and return ttft_ms, total_ms, tokens_per_s."""
    t_start = time.perf_counter()
    t_first: Optional[float] = None
    t_end: Optional[float] = None
    completion_tokens = 0

    stream = client.chat.completions.create(
        model=model,
        messages=messages,
        stream=True,
        stream_options={"include_usage": True},
        max_tokens=max_tokens,
    )

    for chunk in stream:
        now = time.perf_counter()
        delta = chunk.choices[0].delta.content if chunk.choices else None
        if delta and t_first is None:
            t_first = now
        if chunk.usage:
            completion_tokens = chunk.usage.completion_tokens or 0
        if chunk.choices and chunk.choices[0].finish_reason in ("stop", "length"):
            t_end = now

    if t_first is None or t_end is None:
        return {"ttft_ms": None, "total_ms": None, "tokens_per_s": None}

    total_s = t_end - t_start
    ttft_ms = (t_first - t_start) * 1000
    total_ms = total_s * 1000
    tps = (
        (completion_tokens / total_s) if total_s > 0 and completion_tokens > 0 else 0.0
    )

    return {"ttft_ms": ttft_ms, "total_ms": total_ms, "tokens_per_s": tps}


def _parse_structured(
    client: OpenAI, model: str, messages: list, response_format, max_tokens: int = 128
) -> Any:
    """Call with structured output; fall back to json.loads on content if needed."""
    try:
        resp = client.beta.chat.completions.parse(
            model=model,
            messages=messages,
            response_format=response_format,
            max_tokens=max_tokens,
            temperature=0.0,
        )
        content = resp.choices[0].message.content or ""
        if "</think>" in content:
            content = content.split("</think>")[-1].strip()
        return response_format(**json.loads(content))
    except Exception:
        # Fallback: unstructured call + manual parse
        resp = client.chat.completions.create(
            model=model,
            messages=messages,
            max_tokens=max_tokens,
            temperature=0.0,
        )
        content = resp.choices[0].message.content or ""
        if "</think>" in content:
            content = content.split("</think>")[-1].strip()
        return response_format(**json.loads(content))


# ── Task classes ──────────────────────────────────────────────────────────────


class ExtractDataTask:
    name = "extract_data"

    @staticmethod
    def load(test_file: str) -> list:
        with open(test_file) as f:
            return json.load(f)

    @staticmethod
    def run_case(client: OpenAI, model: str, case: list) -> dict:
        text, data_type, context, expected = case
        messages = _extract_data_messages(text, data_type, context)
        try:
            result = _parse_structured(
                client, model, messages, ExtractedData, max_tokens=60
            )
            got = result.data
        except Exception as e:
            got = f"ERROR: {e}"
        passed = got.strip().lower() == str(expected).strip().lower()
        return {"input": case, "expected": expected, "got": got, "passed": passed}

    @staticmethod
    def perf_messages() -> list:
        return _extract_data_messages(
            "My name is Carlos and I would like a glass of water.", "drink"
        )


class IsPositiveTask:
    name = "is_positive"

    @staticmethod
    def load(test_file: str) -> list:
        with open(test_file) as f:
            return json.load(f)

    @staticmethod
    def run_case(client: OpenAI, model: str, case: list) -> dict:
        text, expected = case
        messages = _is_positive_messages(text)
        try:
            result = _parse_structured(
                client, model, messages, IsAnswerPositive, max_tokens=20
            )
            got = result.is_positive
        except Exception as e:
            got = f"ERROR: {e}"
        passed = got == expected if not isinstance(got, str) else False
        return {"input": text, "expected": expected, "got": got, "passed": passed}

    @staticmethod
    def perf_messages() -> list:
        return _is_positive_messages("Yes, that's correct")


class IsNegativeTask:
    name = "is_negative"

    @staticmethod
    def load(test_file: str) -> list:
        with open(test_file) as f:
            return json.load(f)

    @staticmethod
    def run_case(client: OpenAI, model: str, case: list) -> dict:
        text, expected = case
        messages = _is_negative_messages(text)
        try:
            result = _parse_structured(
                client, model, messages, IsAnswerNegative, max_tokens=20
            )
            got = result.is_negative
        except Exception as e:
            got = f"ERROR: {e}"
        passed = got == expected if not isinstance(got, str) else False
        return {"input": text, "expected": expected, "got": got, "passed": passed}

    @staticmethod
    def perf_messages() -> list:
        return _is_negative_messages("No, that's wrong")


class CommandInterpreterTask:
    name = "command_interpreter"

    @staticmethod
    def load(test_file: str) -> list:
        with open(test_file) as f:
            return json.load(f)

    @staticmethod
    def _extract_actions(s: str) -> list:
        return re.findall(r"action='(\w+)'", s)

    @classmethod
    def _jaccard(cls, expected_str: str, got_str: str) -> float:
        exp = cls._extract_actions(expected_str)
        got = cls._extract_actions(got_str)
        if not exp and not got:
            return 1.0
        if not exp or not got:
            return 0.0
        exp_set, got_set = set(exp), set(got)
        return len(exp_set & got_set) / len(exp_set | got_set)

    @classmethod
    def run_case(cls, client: OpenAI, model: str, case: dict) -> dict:
        command = case["string_cmd"]
        expected = case["structured_cmd"]
        messages = _command_interpreter_messages(command)
        try:
            resp = client.chat.completions.create(
                model=model, messages=messages, max_tokens=256, temperature=0.0
            )
            content = resp.choices[0].message.content or ""
            if "</think>" in content:
                content = content.split("</think>")[-1].strip()
            got = content
        except Exception as e:
            got = f"ERROR: {e}"
        score = cls._jaccard(expected, got)
        passed = score >= 0.8
        return {
            "input": command,
            "expected": expected,
            "got": got,
            "jaccard": round(score, 3),
            "passed": passed,
        }

    @staticmethod
    def perf_messages() -> list:
        return _command_interpreter_messages(
            "go to the kitchen, pick up the apple, and bring it to the living room"
        )


def run_perf(client: OpenAI, model: str, task_cls, runs: int) -> dict:
    """Run `runs` timed inferences on the task's representative prompt."""
    msgs = task_cls.perf_messages()
    ttft_list, tps_list = [], []
    for _ in range(runs):
        r = _run_timed(client, model, msgs)
        if r["ttft_ms"] is not None:
            ttft_list.append(r["ttft_ms"])
            tps_list.append(r["tokens_per_s"])
    if not ttft_list:
        return {"avg_ttft_ms": None, "avg_tokens_per_s": None}
    return {
        "avg_ttft_ms": round(sum(ttft_list) / len(ttft_list), 1),
        "avg_tokens_per_s": round(sum(tps_list) / len(tps_list), 1),
    }


TASK_REGISTRY = {
    "extract_data": ExtractDataTask,
    "is_positive": IsPositiveTask,
    "is_negative": IsNegativeTask,
    "command_interpreter": CommandInterpreterTask,
}
