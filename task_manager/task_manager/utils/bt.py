"""Minimal behaviour-tree runtime for task managers: leaves, loops and a logging runner."""

import json
import time
from pathlib import Path
from typing import Any, Callable, Optional

import py_trees
from py_trees.common import Status as BT

from task_manager.utils.status import Status


def _ok(result) -> bool:
    """Map a subtask return value (bool, Status or (Status, ...)) to success."""
    if isinstance(result, bool):
        return result
    if isinstance(result, tuple):
        result = result[0] if result else None
    return result == Status.EXECUTION_SUCCESS


class Action(py_trees.behaviour.Behaviour):
    """Runs fn() once: RUNNING on the first tick so the runner can log/step, executes on the next."""

    hooks = None  # RunLog shared by every leaf, set by TreeRunner

    def __init__(self, name: str, fn: Callable[[], Any]):
        super().__init__(name)
        self.fn = fn
        self._armed = False

    def initialise(self):
        self._armed = False

    def update(self):
        if not self._armed:
            self._armed = True
            if Action.hooks:
                Action.hooks.before(self.name)
            return BT.RUNNING
        start = time.time()
        try:
            result = self.fn()
            ok = _ok(result)
            detail = "" if ok else repr(result)
        except Exception as e:  # noqa: BLE001
            ok, detail = False, f"raised {e!r}"
        if Action.hooks:
            Action.hooks.after(self.name, ok, time.time() - start, detail)
        return BT.SUCCESS if ok else BT.FAILURE


class ForEach(py_trees.behaviour.Behaviour):
    """Builds and runs one subtree per item until next_item() returns None.

    on_done(item, ok) records the outcome so next_item() moves on even when the subtree fails.
    """

    def __init__(self, name, next_item, build, on_done):
        super().__init__(name)
        self.next_item = next_item
        self.build = build
        self.on_done = on_done
        self.item = None
        self.child = None

    def update(self):
        if self.child is None:
            self.item = self.next_item()
            if self.item is None:
                return BT.SUCCESS
            self.child = self.build(self.item)
        self.child.tick_once()
        if self.child.status == BT.RUNNING:
            return BT.RUNNING
        self.on_done(self.item, self.child.status == BT.SUCCESS)
        self.child = None
        return BT.RUNNING

    def terminate(self, new_status):
        if self.child is not None and new_status == BT.INVALID:
            self.child.stop(BT.INVALID)
            self.child = None


def sequence(name: str, *children) -> py_trees.composites.Sequence:
    """Run children in order; stop at the first failure."""
    return py_trees.composites.Sequence(name=name, memory=True, children=list(children))


def optional(child) -> py_trees.decorators.FailureIsSuccess:
    """A failure here does not fail the parent."""
    return py_trees.decorators.FailureIsSuccess(name=f"optional({child.name})", child=child)


class RunLog:
    """Per-leaf JSONL log plus a state snapshot after every leaf."""

    def __init__(self, directory: str, snapshot: Callable[[], dict], on_leaf=None, step=False):
        self.dir = Path(directory).expanduser() / time.strftime("%Y%m%d_%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        self.snapshot = snapshot
        self.on_leaf = on_leaf
        self.step = step
        self.count = 0
        self.totals: dict[str, float] = {}

    def before(self, name: str):
        if self.on_leaf:
            self.on_leaf(name)
        if self.step:
            input(f"[step] next: {name} — press Enter")

    def after(self, name: str, ok: bool, duration: float, detail: str):
        self.count += 1
        self.totals[name] = self.totals.get(name, 0.0) + duration
        row = {"n": self.count, "leaf": name, "ok": ok, "dur_s": round(duration, 2)}
        if detail:
            row["detail"] = detail
        with open(self.dir / "run_log.jsonl", "a") as f:
            f.write(json.dumps(row) + "\n")
        state_file = self.dir / f"state_{self.count:03d}_{name}.json"
        state_file.write_text(json.dumps(self.snapshot(), indent=2, default=str))
        print(f"[BT] {'OK  ' if ok else 'FAIL'} {name} ({duration:.1f}s) {detail}")

    def report(self) -> list[tuple[str, float]]:
        return sorted(self.totals.items(), key=lambda kv: kv[1], reverse=True)


class TreeRunner:
    """Ticks a tree until it finishes or should_stop() is true."""

    def __init__(self, root, log: RunLog, should_stop: Optional[Callable[[], bool]] = None):
        self.root = root
        self.log = log
        self.should_stop = should_stop or (lambda: False)

    def run(self, spin: Callable[[], None] = lambda: None) -> BT:
        Action.hooks = self.log
        try:
            while True:
                if self.should_stop():
                    self.root.stop(BT.INVALID)
                    return BT.INVALID
                self.root.tick_once()
                if self.root.status != BT.RUNNING:
                    return self.root.status
                spin()
        finally:
            Action.hooks = None
