"""
Behaviour-tree decorators for competition time budgets.

py_trees' own Timeout takes a duration measured from when the child starts. A task
needs the other thing: an absolute wall-clock budget anchored at door-open, checked no
matter which action happens to be running. That is ``Deadline``.
"""

import time
from typing import Callable, Optional

import py_trees


class Deadline(py_trees.decorators.Decorator):
    """
    Fail the subtree once the run's time budget is spent.

    Args:
        child: the subtree to guard.
        remaining_s: callable returning the seconds left in the run.
        name: behaviour name.
        on_expire: optional callback fired once, when the deadline first trips.
    """

    def __init__(
        self,
        child: py_trees.behaviour.Behaviour,
        remaining_s: Callable[[], float],
        name: str = "deadline",
        on_expire: Optional[Callable[[], None]] = None,
    ):
        super().__init__(name=name, child=child)
        self._remaining_s = remaining_s
        self._on_expire = on_expire
        self._expired = False

    def update(self) -> py_trees.common.Status:
        if self._expired or self._remaining_s() <= 0.0:
            if not self._expired:
                self._expired = True
                self.logger.warning(f"{self.name}: time budget exhausted")
                if self._on_expire is not None:
                    try:
                        self._on_expire()
                    except Exception:  # noqa: BLE001 — never let a hook tank the tree
                        self.logger.error(f"{self.name} on_expire hook raised")
            # stop the child so its leaf can abandon any in-flight skill call
            if self.decorated.status == py_trees.common.Status.RUNNING:
                self.decorated.stop(py_trees.common.Status.INVALID)
            return py_trees.common.Status.FAILURE
        return self.decorated.status


class Budget:
    """Wall-clock budget anchored at an explicit start, for use with ``Deadline``."""

    def __init__(self, total_s: float):
        self.total_s = total_s
        self._t0: Optional[float] = None

    def start(self) -> None:
        """Anchor the budget. Call at door-open, not at node construction."""
        self._t0 = time.time()

    def elapsed(self) -> float:
        return 0.0 if self._t0 is None else time.time() - self._t0

    def remaining(self) -> float:
        """Seconds left; the full budget while unanchored so nothing trips early."""
        return self.total_s if self._t0 is None else self.total_s - self.elapsed()

    def expired(self) -> bool:
        return self.remaining() <= 0.0
