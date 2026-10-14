"""Recovery supervisors: decide what to do after a failed skill attempt.

RulesSupervisor reproduces what the hand-written FSMs do today (retry up to
max_attempts, then the deus-ex-machina fallback, then move on).
LLMSupervisor asks an OpenAI-compatible server (llama.cpp / ollama) and falls
back to the rules on timeout, network error or an invalid answer, so the robot
never blocks on the model.
"""

import json
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .events import ABORT, ACTIONS, FALLBACK, RETRY, SKIP, Event
from .spec import Step

# Guard rail: the LLM may retry a step at most retry_cap_factor * max_attempts
# times. 1.0 (default) means it can only stop retrying earlier than the rules.
DEFAULT_RETRY_CAP_FACTOR = 1.0
MAX_SAY_CHARS = 200


@dataclass
class DecisionContext:
    task: str
    step: Step
    error: str
    attempts: int  # attempts made with the primary skill
    fallbacks_used: int
    elapsed_s: float
    remaining_s: float
    points_so_far: float
    points_at_stake: float  # points of this step
    remaining_points: float  # points of the steps still ahead (incl. this one)
    upcoming: List[str]
    recent: List[Event]
    blackboard: Dict[str, Any] = field(default_factory=dict)

    @property
    def fallbacks_left(self) -> List[str]:
        return self.step.fallbacks[self.fallbacks_used :]


@dataclass
class Decision:
    action: str
    say: str = ""
    reason: str = ""
    source: str = "rules"
    latency_s: float = 0.0
    valid: bool = True


class RulesSupervisor:
    name = "rules"

    def decide(self, ctx: DecisionContext) -> Decision:
        if ctx.fallbacks_used == 0 and ctx.attempts < ctx.step.max_attempts:
            return Decision(RETRY, reason=f"attempt {ctx.attempts}/{ctx.step.max_attempts}")
        if ctx.fallbacks_left:
            return Decision(FALLBACK, reason="primary attempts exhausted")
        return Decision(SKIP, reason="no attempts or fallbacks left")


class ErrorAwareRulesSupervisor(RulesSupervisor):
    """Rules that stop retrying when the error matches a known permanent pattern
    (spec ``permanent_errors``). Baseline to check whether the LLM adds value
    beyond a better deterministic rule."""

    name = "rules+"

    def __init__(self, permanent_errors: Optional[List[str]] = None) -> None:
        self.patterns = [p.lower() for p in (permanent_errors or [])]

    def decide(self, ctx: DecisionContext) -> Decision:
        error = ctx.error.lower()
        if ctx.fallbacks_used == 0 and any(p in error for p in self.patterns):
            if ctx.fallbacks_left:
                return Decision(FALLBACK, reason="permanent error pattern")
            return Decision(SKIP, reason="permanent error pattern, no fallback")
        return super().decide(ctx)


SYSTEM_PROMPT = """{persona}

You supervise the robot during the RoboCup@Home task "{task}". A step just failed.
Choose ONE action:
- "retry": try the same skill again. Good when the error looks transient (noise, timing, person moved).
- "fallback": use the next fallback ({fallbacks}). Usually asking a human for help; it scores fewer points.
- "skip": give up this step and continue with the next ones. Good when the error looks permanent or time is short.
- "abort": stop the whole task. Almost never correct.
Think about: is the error transient or permanent? How much time is left versus what the
remaining steps are worth? Never retry the same permanent error.
Skipping loses ALL the points of the step and a fallback still scores some, so only skip when
there is no fallback left or the remaining time cannot cover the remaining steps (roughly 60s
per step). If an error is not listed as permanent in the rules, assume it is transient.
Task rules:
{rules}

Answer only JSON: {{"action": ..., "say": ..., "reason": ...}}.
"say" is ONE short sentence the robot says aloud to the people around (friendly, may use light humour, never blame people), or "" to stay silent.
"reason" is a few words for the log."""

DECISION_SCHEMA = {
    "type": "object",
    "properties": {
        "action": {"type": "string", "enum": list(ACTIONS)},
        "say": {"type": "string"},
        "reason": {"type": "string"},
    },
    "required": ["action", "say", "reason"],
}


def _format_state(ctx: DecisionContext) -> str:
    lines = [
        f"Failed step: {ctx.step.id} (skill {ctx.step.skill}, args {ctx.step.args})",
        f"Error: {ctx.error}",
        f"Attempts with primary skill: {ctx.attempts} (planned max {ctx.step.max_attempts})",
        f"Fallbacks left: {ctx.fallbacks_left or 'none'}",
        f"Time: {ctx.elapsed_s:.0f}s used, {ctx.remaining_s:.0f}s left",
        f"Points: {ctx.points_so_far:.0f} scored, this step worth {ctx.points_at_stake:.0f}, "
        f"{ctx.remaining_points:.0f} still possible in total",
        f"Next steps: {', '.join(ctx.upcoming[:6]) or 'none'}",
        "Recent log:",
    ]
    lines += [f"  {e.short()}" for e in ctx.recent]
    return "\n".join(lines)


def _extract_json(text: str) -> Optional[Dict[str, Any]]:
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL).strip()
    match = re.search(r"\{.*\}", text, flags=re.DOTALL)
    if not match:
        return None
    try:
        return json.loads(match.group(0))
    except json.JSONDecodeError:
        return None


class LLMSupervisor:
    name = "llm"

    def __init__(
        self,
        base_url: str = "http://localhost:11434/v1",
        model: str = "frida-llm",
        persona: str = "",
        rules: str = "",
        timeout_s: float = 8.0,
        temperature: float = 0.2,
        max_tokens: int = 120,
        recent_events: int = 8,
        api_key: str = "none",
        retry_cap_factor: float = DEFAULT_RETRY_CAP_FACTOR,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.persona = persona
        self.rules = rules
        self.timeout_s = timeout_s
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.recent_events = recent_events
        self.api_key = api_key
        self.retry_cap_factor = retry_cap_factor
        self.fallback = RulesSupervisor()

    # ── transport ──
    def _chat(self, messages: List[Dict[str, str]]) -> str:
        body = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
            # llama.cpp turns this schema into a grammar -> always parseable JSON
            "response_format": {
                "type": "json_schema",
                "json_schema": {"name": "decision", "schema": DECISION_SCHEMA},
            },
        }
        req = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=json.dumps(body).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}",
            },
        )
        with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
        return payload["choices"][0]["message"]["content"] or ""

    def build_messages(self, ctx: DecisionContext) -> List[Dict[str, str]]:
        system = SYSTEM_PROMPT.format(
            persona=self.persona or "You are FRIDA, a domestic service robot.",
            task=ctx.task,
            fallbacks=", ".join(ctx.step.fallbacks) or "none",
            rules=self.rules.strip() or "(none)",
        )
        # "/no_think" disables Qwen3's reasoning block: latency matters more here.
        return [
            {"role": "system", "content": system},
            {"role": "user", "content": _format_state(ctx) + "\n/no_think"},
        ]

    # ── decision ──
    def _validate(self, ctx: DecisionContext, data: Optional[Dict[str, Any]]) -> Optional[str]:
        """Return an error string if the answer can't be executed as-is."""
        if not data or data.get("action") not in ACTIONS:
            return f"bad action: {data}"
        action = data["action"]
        if action == FALLBACK and not ctx.fallbacks_left:
            return "fallback requested but none left"
        if action == RETRY and ctx.attempts >= ctx.step.max_attempts * self.retry_cap_factor:
            return "retry cap reached"
        if action in (SKIP, ABORT) and not ctx.step.skippable:
            return f"{action} not allowed on step {ctx.step.id}"
        if action == RETRY and ctx.fallbacks_used > 0:
            return "retry after fallback is not allowed"
        return None

    def decide(self, ctx: DecisionContext) -> Decision:
        start = time.monotonic()
        try:
            raw = self._chat(self.build_messages(ctx))
            data = _extract_json(raw)
            error = self._validate(ctx, data)
        except (urllib.error.URLError, TimeoutError, OSError, KeyError, ValueError) as e:
            data, error = None, f"llm error: {e}"
        latency = time.monotonic() - start

        if error:
            fallback = self.fallback.decide(ctx)
            fallback.source = "rules(llm-fallback)"
            fallback.reason = f"{error} | {fallback.reason}"
            fallback.latency_s = latency
            fallback.valid = False
            return fallback

        return Decision(
            action=data["action"],
            say=str(data.get("say", ""))[:MAX_SAY_CHARS],
            reason=str(data.get("reason", "")),
            source="llm",
            latency_s=latency,
        )


def make_supervisor(policy: str, permanent_errors: Optional[List[str]] = None, **llm_kwargs) -> Any:
    if policy == "rules":
        return RulesSupervisor()
    if policy == "rules+":
        return ErrorAwareRulesSupervisor(permanent_errors)
    if policy == "llm":
        return LLMSupervisor(**llm_kwargs)
    raise ValueError(f"Unknown policy '{policy}' (expected rules|rules+|llm)")


__all__ = [
    "ABORT",
    "Decision",
    "DecisionContext",
    "ErrorAwareRulesSupervisor",
    "LLMSupervisor",
    "RulesSupervisor",
    "SKIP",
    "make_supervisor",
]
