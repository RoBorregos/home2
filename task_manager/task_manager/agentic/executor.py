"""Runs a TaskSpec: executes skills step by step, keeps the time budget and asks
the supervisor what to do whenever a skill fails."""

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Tuple

from .events import ABORT, FALLBACK, RETRY, Event, EventLog, SkillResult
from .spec import Loop, Node, Step, TaskSpec, format_args
from .supervisor import Decision, DecisionContext

Skill = Callable[..., SkillResult]


class WallClock:
    def now(self) -> float:
        return time.monotonic()


class SimClock:
    """Simulated time: skills advance it by their modelled duration."""

    def __init__(self) -> None:
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


@dataclass
class TaskResult:
    task: str
    policy: str
    points: float = 0.0
    max_points: float = 0.0
    elapsed_s: float = 0.0
    timed_out: bool = False
    aborted: bool = False
    outcomes: Dict[str, str] = field(default_factory=dict)  # step -> success|fallback|skipped
    decisions: List[Decision] = field(default_factory=list)
    fallbacks_run: int = 0
    attempts: int = 0
    utterances: List[str] = field(default_factory=list)
    events: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def llm_latencies(self) -> List[float]:
        return [d.latency_s for d in self.decisions if d.source != "rules"]

    @property
    def invalid_decisions(self) -> int:
        return sum(1 for d in self.decisions if not d.valid)


class TaskExecutor:
    def __init__(
        self,
        spec: TaskSpec,
        skills: Dict[str, Skill],
        supervisor: Any,
        clock: Any = None,
        log: Callable[[str], None] = print,
        say_skill: str = "say",
    ) -> None:
        self.spec = spec
        self.skills = skills
        self.supervisor = supervisor
        self.clock = clock or WallClock()
        self.log = log
        self.say_skill = say_skill
        self.blackboard: Dict[str, Any] = dict(spec.blackboard)
        self.events = EventLog()
        self._t0 = 0.0
        # pending work: (node, loop bindings, instance-id prefix)
        self._queue: List[Tuple[Node, Dict[str, Any], str]] = []

    # ── helpers ──
    def elapsed(self) -> float:
        return self.clock.now() - self._t0

    def remaining(self) -> float:
        return self.spec.time_limit_s - self.elapsed()

    def _call(self, skill_name: str, args: Dict[str, Any]) -> SkillResult:
        skill = self.skills.get(skill_name)
        if skill is None:
            return SkillResult.fail(f"unknown skill '{skill_name}'")
        try:
            result = skill(**args)
        except Exception as e:  # a crashing skill is just a failed attempt
            return SkillResult.fail(f"exception: {e}")
        if not isinstance(result, SkillResult):
            return SkillResult.fail(f"skill returned {type(result).__name__}")
        return result

    def _say(self, text: str, result: TaskResult) -> None:
        if not text:
            return
        result.utterances.append(text)
        if self.say_skill in self.skills:
            self._call(self.say_skill, {"text": text})

    def _expand(self, node: Node, bindings: Dict[str, Any], prefix: str):
        """Turn a node into concrete (step, bindings, prefix) entries; loops use
        the current blackboard list."""
        if isinstance(node, Step):
            return [(node, bindings, prefix)]
        items = list(self.blackboard.get(node.foreach, []) or [])
        out = []
        for i, item in enumerate(items):
            inner = dict(bindings, **{node.var: item})
            for child in node.steps:
                out += self._expand(child, inner, f"{prefix}{node.foreach}[{i}].")
        return out

    def _upcoming(self) -> Tuple[List[str], float]:
        ids, points = [], 0.0
        for node, bindings, prefix in self._queue:
            for step, _, step_prefix in self._expand(node, bindings, prefix):
                ids.append(step_prefix + step.id)
                points += step.points
        return ids, points

    # ── main loop ──
    def run(self) -> TaskResult:
        result = TaskResult(task=self.spec.name, policy=getattr(self.supervisor, "name", "?"))
        self._t0 = self.clock.now()
        self._queue = [(node, {}, "") for node in self.spec.steps]

        while self._queue:
            if self.remaining() <= 0:
                result.timed_out = True
                self.log(f"[{self.spec.name}] time limit reached")
                break
            node, bindings, prefix = self._queue.pop(0)
            if isinstance(node, Loop):
                self._queue = self._expand(node, bindings, prefix) + self._queue
                continue
            if not self._run_step(node, bindings, prefix + node.id, result):
                break

        result.elapsed_s = self.elapsed()
        result.max_points = self.spec.max_points(self.blackboard)
        result.events = self.events.as_dicts()
        return result

    def _run_step(
        self, step: Step, bindings: Dict[str, Any], inst: str, result: TaskResult
    ) -> bool:
        """Execute one step with supervised recovery. Returns False to stop the task."""
        scope = dict(self.blackboard, **bindings)
        args = format_args(step.args, scope)
        attempts, fallbacks_used = 0, 0
        skill_name = step.skill

        while True:
            if fallbacks_used == 0:
                attempts += 1
            result.attempts += 1
            res = self._call(skill_name, args)
            event = self.events.add(
                Event(
                    t=self.elapsed(),
                    step=inst,
                    skill=skill_name,
                    attempt=attempts if fallbacks_used == 0 else fallbacks_used,
                    success=res.success,
                    error=res.error,
                )
            )
            if res.success:
                self.blackboard.update(res.data)
                if fallbacks_used:
                    gained = (
                        step.fallback_points
                        if step.fallback_points is not None
                        else step.points / 2
                    )
                    result.outcomes[inst] = "fallback"
                else:
                    gained = step.points
                    result.outcomes[inst] = "success"
                result.points += gained
                self.log(f"[{inst}] {skill_name} ok (+{gained:.0f})")
                return True

            self.log(f"[{inst}] {skill_name} failed: {res.error}")
            if self.remaining() <= 0:
                result.outcomes[inst] = "skipped"
                return True  # the main loop reports the timeout

            upcoming, upcoming_points = self._upcoming()
            ctx = DecisionContext(
                task=self.spec.name,
                step=step,
                error=res.error,
                attempts=attempts,
                fallbacks_used=fallbacks_used,
                elapsed_s=self.elapsed(),
                remaining_s=self.remaining(),
                points_so_far=result.points,
                points_at_stake=step.points,
                remaining_points=upcoming_points + step.points,
                upcoming=upcoming,
                recent=self.events.tail(),
                blackboard=scope,
            )
            decision = self.supervisor.decide(ctx)
            result.decisions.append(decision)
            if hasattr(self.clock, "advance"):
                # thinking time is task time too
                self.clock.advance(decision.latency_s)
            event.decision, event.decision_source, event.reason = (
                decision.action,
                decision.source,
                decision.reason,
            )
            self.log(f"[{inst}] decision={decision.action} ({decision.source}): {decision.reason}")
            self._say(decision.say, result)

            if decision.action == RETRY:
                skill_name = step.skill
            elif decision.action == FALLBACK and ctx.fallbacks_left:
                skill_name = ctx.fallbacks_left[0]
                fallbacks_used += 1
                result.fallbacks_run += 1
            elif decision.action == ABORT:
                result.aborted = True
                result.outcomes[inst] = "skipped"
                return False
            else:  # SKIP (or an impossible fallback)
                result.outcomes[inst] = "skipped"
                if step.critical:
                    result.aborted = True
                    return False
                return True
