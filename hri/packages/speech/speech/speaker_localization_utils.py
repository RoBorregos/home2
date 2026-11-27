"""Who + where: direction lookup by time and speaker tracker."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from speech.mic_array_utils import circ_diff, merge_doas


class DirectionBuffer:
    def __init__(self, history_sec: float = 20.0, slack_sec: float = 0.1) -> None:
        self.history_sec = history_sec
        self.slack_sec = slack_sec
        self._items: deque[dict] = deque()

    def add(self, est: dict) -> None:
        self._items.append(est)
        while self._items and est["t"] - self._items[0]["t"] > self.history_sec:
            self._items.popleft()

    def between(self, t_start: float, t_end: float) -> list[dict]:
        out = [
            e
            for e in self._items
            if t_start + 0.5 * e.get("window_sec", 0.0) <= e["t"]
            and e["t"] <= t_end + self.slack_sec
        ]
        if out or not self._items:
            return out
        near = min(self._items, key=lambda e: abs(e["t"] - t_end))
        return [near] if abs(near["t"] - t_end) <= 0.5 else []

    def direction(self, spans: list, min_conf: float) -> dict | None:
        ests = {id(e): e for a, b in spans for e in self.between(a, b)}
        return merge_doas(list(ests.values()), min_conf)


@dataclass
class Track:
    name: str
    azimuth_deg: float
    last_t: float
    hits: int = 1


@dataclass
class TrackerConfig:
    max_age_sec: float = 8.0
    match_deg: float = 25.0
    smooth: float = 0.5


class SpeakerTracker:
    def __init__(self, cfg: TrackerConfig | None = None):
        self.cfg = cfg or TrackerConfig()
        self.tracks: dict[str, Track] = {}

    def _fresh(self, now: float) -> list[Track]:
        return [
            t for t in self.tracks.values() if now - t.last_t <= self.cfg.max_age_sec
        ]

    def update(
        self,
        now: float,
        name: str | None,
        confident: bool,
        azimuth_deg: float | None,
        doa_reliable: bool = True,
        rejected: bool = False,
    ) -> tuple[str, str]:
        if confident and name:
            if azimuth_deg is not None and doa_reliable:
                tr = self.tracks.get(name)
                if tr is None or now - tr.last_t > self.cfg.max_age_sec:
                    self.tracks[name] = Track(name, azimuth_deg, now)
                else:
                    step = self.cfg.smooth * float(
                        circ_diff(azimuth_deg, tr.azimuth_deg)
                    )
                    tr.azimuth_deg = float(circ_diff(tr.azimuth_deg + step, 0.0))
                    tr.last_t = now
                    tr.hits += 1
            elif name in self.tracks:
                self.tracks[name].last_t = now
            return name, "voice"
        if rejected or azimuth_deg is None or not doa_reliable:
            return "unknown", "none"
        best, best_d = None, None
        for tr in self._fresh(now):
            d = abs(float(circ_diff(azimuth_deg, tr.azimuth_deg)))
            if d <= self.cfg.match_deg and (best_d is None or d < best_d):
                best, best_d = tr, d
        if best is None:
            return "unknown", "none"
        return best.name, "direction"

    def snapshot(self, now: float) -> list[dict]:
        return [
            {
                "name": t.name,
                "azimuth_deg": round(t.azimuth_deg, 1),
                "age_sec": round(now - t.last_t, 1),
                "hits": t.hits,
            }
            for t in self._fresh(now)
        ]
