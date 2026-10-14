"""Skill results and the structured event log shared by executor and supervisor."""

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional

# Decision actions the supervisor may return after a failed attempt.
RETRY = "retry"
FALLBACK = "fallback"
SKIP = "skip"
ABORT = "abort"
ACTIONS = (RETRY, FALLBACK, SKIP, ABORT)


@dataclass
class SkillResult:
    success: bool
    error: str = ""
    # Values to merge into the blackboard (e.g. {"orders": [...]}).
    data: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def ok(cls, **data) -> "SkillResult":
        return cls(success=True, data=data)

    @classmethod
    def fail(cls, error: str, **data) -> "SkillResult":
        return cls(success=False, error=error, data=data)


@dataclass
class Event:
    t: float
    step: str
    skill: str
    attempt: int
    success: bool
    error: str = ""
    decision: Optional[str] = None
    decision_source: Optional[str] = None
    reason: str = ""

    def short(self) -> str:
        """One compact line for the LLM context window."""
        outcome = "ok" if self.success else f"FAIL({self.error})"
        line = f"t={self.t:.0f}s {self.step}/{self.skill}#{self.attempt} {outcome}"
        if self.decision:
            line += f" -> {self.decision}"
        return line


class EventLog:
    def __init__(self) -> None:
        self.events: List[Event] = []

    def add(self, event: Event) -> Event:
        self.events.append(event)
        return event

    def tail(self, n: int = 8) -> List[Event]:
        return self.events[-n:]

    def as_dicts(self) -> List[Dict[str, Any]]:
        return [asdict(e) for e in self.events]
