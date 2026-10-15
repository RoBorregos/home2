"""
Experiment metrics: wraps subtask manager methods (HRI, vision, manipulation) on the
instance to count/time them, tracks how long a face was being followed, and writes
one JSON line per participant session (metrics + transcript + survey answers).
"""

import json
import os
import time
from contextlib import contextmanager
from datetime import datetime

from task_manager.utils.status import Status

HRI_METHODS = (
    "say",
    "hear",
    "confirm",
    "ask_and_confirm",
    "extract_data",
    "interpret_keyword",
    "answer_with_context",
    "find_closest",
)

VISION_METHODS = (
    "detect_person",
    "activate_face_recognition",
    "deactivate_face_recognition",
    "follow_by_name",
)

MANIPULATION_METHODS = ("follow_face", "move_to_position")

# A face counts as followed during a bucket if at least one follow message arrived in it
FACE_BUCKET_S = 0.5


def _status_name(result) -> str:
    status = result[0] if isinstance(result, tuple) and result else result
    return status.name if isinstance(status, Status) else type(status).__name__


class DialogueMetrics:
    def __init__(self, results_dir: str, experiment: str, label: str):
        os.makedirs(results_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_path = os.path.join(results_dir, f"{label}_{stamp}.jsonl")
        self.experiment = experiment
        self.label = label
        self.session = None
        self.session_count = 0
        self._t0 = 0.0
        self._confidences = []
        self._face_times = []
        self._face_start = None

    def instrument(self, target, methods=HRI_METHODS) -> None:
        """Replace methods on this instance. Nested calls (e.g. the confirm inside
        ask_and_confirm) resolve to the instance attribute, so they are counted too."""
        for name in methods:
            if hasattr(target, name):
                setattr(target, name, self._wrap(name, getattr(target, name)))

    def _wrap(self, name, method):
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            result = method(*args, **kwargs)
            self._record_call(name, time.perf_counter() - start, result)
            return result

        return wrapper

    def _record_call(self, name, elapsed, result) -> None:
        if self.session is None:
            return
        call = self.session["calls"].setdefault(name, {"count": 0, "total_s": 0.0, "statuses": {}})
        call["count"] += 1
        call["total_s"] = round(call["total_s"] + elapsed, 3)
        status = _status_name(result)
        call["statuses"][status] = call["statuses"].get(status, 0) + 1

        if name == "hear" and isinstance(result, tuple) and len(result) == 3:
            _, text, confidences = result
            self.transcript("user", text)
            if confidences:
                self._confidences.extend(confidences.values())

    def start_face_window(self) -> None:
        """Start measuring face following (call right after follow_face(True))."""
        if self.session is not None:
            self._face_start = time.perf_counter()
            self._face_times = []

    def face_seen(self) -> None:
        """Call on every follow-face message."""
        if self.session is not None and self._face_start is not None:
            self._face_times.append(time.perf_counter())

    def _face_stats(self) -> dict | None:
        if self._face_start is None:
            return None
        window = time.perf_counter() - self._face_start
        buckets = max(1, int(window / FACE_BUCKET_S))
        seen = {int((t - self._face_start) / FACE_BUCKET_S) for t in self._face_times}
        gaps = [b - a for a, b in zip(self._face_times, self._face_times[1:])]
        return {
            "window_s": round(window, 2),
            "messages": len(self._face_times),
            "rate_hz": round(len(self._face_times) / window, 2) if window > 0 else 0.0,
            "tracked_ratio": round(min(1.0, len(seen) / buckets), 3),
            "first_face_s": (
                round(self._face_times[0] - self._face_start, 2) if self._face_times else None
            ),
            "max_gap_s": round(max(gaps), 2) if gaps else None,
        }

    def start_session(self, **fields) -> None:
        self.session_count += 1
        self._t0 = time.perf_counter()
        self._confidences = []
        self._face_times = []
        self._face_start = None
        self.session = {
            "experiment": self.experiment,
            "label": self.label,
            "session": self.session_count,
            "started_at": datetime.now().isoformat(timespec="seconds"),
            "outcome": None,
            "person_detected": None,
            "data_sources": {},
            "survey": None,
            "phases": {},
            "calls": {},
            "transcripts": [],
            **fields,
        }

    @property
    def active(self) -> bool:
        return self.session is not None

    def note(self, **fields) -> None:
        if self.session is not None:
            self.session.update(fields)

    def append(self, field: str, value) -> None:
        if self.session is not None:
            self.session.setdefault(field, []).append(value)

    def count_source(self, source: str | None) -> None:
        """Record which data source (online / backup) answered a schedule call."""
        if self.session is not None and source:
            sources = self.session["data_sources"]
            sources[source] = sources.get(source, 0) + 1

    def transcript(self, role: str, text: str) -> None:
        if self.session is not None:
            self.session["transcripts"].append(
                {"t": round(time.perf_counter() - self._t0, 2), "role": role, "text": text}
            )

    @contextmanager
    def phase(self, name: str):
        start = time.perf_counter()
        try:
            yield
        finally:
            if self.session is not None:
                phases = self.session["phases"]
                phases[name] = round(phases.get(name, 0.0) + time.perf_counter() - start, 3)

    def end_session(self, outcome: str) -> dict:
        session = self.session
        if session is None:
            return {}
        session["outcome"] = outcome
        session["duration_s"] = round(time.perf_counter() - self._t0, 3)
        session["stt_confidence_avg"] = (
            round(sum(self._confidences) / len(self._confidences), 3) if self._confidences else None
        )
        session["face_following"] = self._face_stats()
        with open(self.out_path, "a") as file:
            file.write(json.dumps(session, ensure_ascii=False) + "\n")
        self.session = None
        return session
