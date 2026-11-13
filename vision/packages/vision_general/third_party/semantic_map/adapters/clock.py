"""Clock adapters. Satisfy the Clock port."""

from __future__ import annotations

import time


class SystemClock:
    """Wall clock, epoch seconds."""

    def now(self) -> float:
        return time.time()


class ManualClock:
    """A clock advanced by hand, so staleness is testable."""

    def __init__(self, start: float = 0.0) -> None:
        self.t = start

    def now(self) -> float:
        return self.t

    def advance(self, seconds: float) -> float:
        """Move time forward and return the new value."""
        self.t += seconds
        return self.t
