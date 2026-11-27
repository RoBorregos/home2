"""Deadline-aware step scheduling shared by every task manager.

A task is a chain of steps (the FSM states, repeated per guest/command). Each step has
an estimated duration and a priority. Before spending time on optional work (a dialog
retry, a confirmation, later a navigation retry) the caller asks `allows(priority, cost)`:
it is granted only if, after paying `cost`, there is still time for every remaining
step that is MORE important. So LOW work is dropped first, HIGH next, MUST never.
Extra work inside a step (a retry, a search loop, a wait) passes `extra=True`: it must also
leave time for the remaining steps AS important, so retries never starve the later steps.
"""

import time
from dataclasses import dataclass
from enum import IntEnum


class Priority(IntEnum):
    MUST = 0  # never skipped: the task (or its points) cannot continue without it
    HIGH = 1  # skipped only to protect MUST steps
    LOW = 2  # first to go: nice-to-have dialog, extra retries, confirmations


@dataclass
class Step:
    name: str
    estimate: float  # seconds, take it from the timing report of real runs
    priority: Priority = Priority.HIGH


class TimeBudget:
    def __init__(self, limit: float, chain: list[Step], clock=time.monotonic):
        self.limit = limit
        self.chain = chain
        self.clock = clock
        self.start = clock()
        self.cursor = -1  # index in chain of the step being executed
        self.step_start = self.start

    def elapsed(self) -> float:
        return self.clock() - self.start

    def remaining(self) -> float:
        return self.limit - self.elapsed()

    def enter(self, name: str) -> float:
        """Move to the next occurrence of `name` in the chain; returns the drift in seconds
        (>0 = behind the plan). Re-entering the current step (a retry) or an unknown name
        keeps the cursor, so a retry never jumps to the same state of a later guest/command."""
        if self.cursor < 0 or self.chain[self.cursor].name != name:
            for i in range(self.cursor + 1, len(self.chain)):
                if self.chain[i].name == name:
                    self.cursor = i
                    self.step_start = self.clock()
                    break
        return self.elapsed() - sum(s.estimate for s in self.chain[: max(self.cursor, 0)])

    def reserve(self, priority: Priority, extra: bool = False) -> float:
        """Seconds needed by the remaining steps more important than `priority`
        (or as important, with `extra`)."""
        return sum(
            s.estimate
            for s in self.chain[self.cursor + 1 :]
            if s.priority < priority or (extra and s.priority == priority)
        )

    def allows(self, priority: Priority, cost: float = 0.0, extra: bool = False) -> bool:
        if priority == Priority.MUST:
            return True
        return self.remaining() - cost >= self.reserve(priority, extra)

    def affords(self, name: str) -> bool:
        """Whether the next step `name` still fits; the caller skips the whole step if not."""
        for step in self.chain[self.cursor + 1 :]:
            if step.name == name:
                return self.allows(step.priority, step.estimate)
        return True

    def behind(self) -> bool:
        """True when the plan for the rest of the chain no longer fits in the time left."""
        current = self.chain[self.cursor] if self.cursor >= 0 else None
        left_in_step = max(current.estimate - (self.clock() - self.step_start), 0) if current else 0
        return self.remaining() < left_in_step + sum(
            s.estimate for s in self.chain[self.cursor + 1 :]
        )
