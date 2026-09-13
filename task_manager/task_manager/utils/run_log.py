"""
Append-only JSONL log of every instrumented skill call, grouped by run.

Pure stdlib on purpose: offline tooling (fit_capabilities, replay_selector) imports
this without a ROS environment. Nothing here may raise into the robot control path.
"""

import json
import os
import threading
import time
from datetime import datetime
from typing import Any, Optional

DEFAULT_LOG_DIR = os.path.expanduser("~/frida_runs")
LOG_DIR_ENV = "FRIDA_RUN_LOG_DIR"


def _log_dir() -> str:
    return os.environ.get(LOG_DIR_ENV, DEFAULT_LOG_DIR)


class RunLog:
    """Process-wide singleton holding the current run id and its JSONL file."""

    # reentrant: start()/finish() hold the lock and call _write(), which takes it again
    _lock = threading.RLock()
    _run_id: Optional[str] = None
    _path: Optional[str] = None
    _t0: Optional[float] = None
    _enabled: bool = True

    @classmethod
    def start(cls, task: str) -> Optional[str]:
        """Open a new run. Call at the real start of the test (door open), not at __init__."""
        with cls._lock:
            try:
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cls._run_id = f"{task}_{stamp}"
                directory = _log_dir()
                os.makedirs(directory, exist_ok=True)
                cls._path = os.path.join(directory, f"{cls._run_id}.jsonl")
                cls._t0 = time.time()
                cls._write({"event": "run_start", "task": task, "run_id": cls._run_id, "t": 0.0})
                return cls._run_id
            except Exception:
                cls._enabled = False
                return None

    @classmethod
    def finish(cls, score: Optional[int] = None) -> None:
        """Close the run, optionally recording the score the team judged it earned."""
        cls._write({"event": "run_end", "t": cls.elapsed(), "score": score})
        with cls._lock:
            cls._run_id = None
            cls._path = None
            cls._t0 = None

    @classmethod
    def elapsed(cls) -> float:
        """Seconds since the run started, 0.0 if no run is open."""
        return time.time() - cls._t0 if cls._t0 is not None else 0.0

    @classmethod
    def run_id(cls) -> Optional[str]:
        return cls._run_id

    @classmethod
    def record(
        cls,
        skill: str,
        status: Any,
        duration_s: float,
        context: Optional[dict] = None,
        result: Any = None,
    ) -> None:
        """Record one skill call outcome."""
        cls._write(
            {
                "event": "skill",
                "t": cls.elapsed(),
                "skill": skill,
                "status": _status_name(status),
                "duration_s": round(duration_s, 3),
                "context": context or {},
                "result": _safe_repr(result),
            }
        )

    @classmethod
    def note(cls, kind: str, **fields: Any) -> None:
        """Record a non-skill event (state change, objective selected, deadline hit)."""
        cls._write({"event": kind, "t": cls.elapsed(), **fields})

    @classmethod
    def _write(cls, row: dict) -> None:
        if not cls._enabled or cls._path is None:
            return
        with cls._lock:
            try:
                row.setdefault("run_id", cls._run_id)
                # flush per line so a crash or power-off keeps everything already recorded
                with open(cls._path, "a", encoding="utf-8") as handle:
                    handle.write(json.dumps(row, default=str) + "\n")
            except Exception:
                cls._enabled = False


def _status_name(status: Any) -> str:
    """Normalize the repo's heterogeneous outcome types to a stable string."""
    name = getattr(status, "name", None)
    if name is not None:
        return str(name)
    if isinstance(status, bool):
        return "EXECUTION_SUCCESS" if status else "EXECUTION_ERROR"
    return str(status)


def _safe_repr(value: Any, limit: int = 200) -> Optional[str]:
    if value is None:
        return None
    try:
        text = str(value)
    except Exception:
        return "<unrepresentable>"
    return text if len(text) <= limit else text[:limit] + "…"
