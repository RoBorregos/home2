"""Associate voice identities with directions over time.

The speaker embedding is unsure on short, turned-away or far phrases (UNKNOWN
band between REJECT_TH and ACCEPT_TH). People rarely teleport, so a voice that
comes from where a known person was confidently identified a moment ago is
most likely still that person. The tracker keeps one track per identified
person (name, azimuth, last confident time) and resolves each decision:

    voice confident (TARGET-level score)      -> that name, via "voice";
                                                 the track moves to this DOA
    voice unsure, DOA near a fresh track      -> track's name, via "direction"
    otherwise                                 -> "unknown"

Only confident voice decisions create or move tracks, so a wrong direction
match cannot snowball into more wrong matches.
"""

from __future__ import annotations

from dataclasses import dataclass

from speech.spatial.doa import circ_diff


@dataclass
class Track:
    name: str
    azimuth_deg: float
    last_t: float
    hits: int = 1


@dataclass
class TrackerConfig:
    max_age_sec: float = 8.0  # a track is usable this long after its last hit
    match_deg: float = 25.0  # DOA gate for attributing an unsure voice
    smooth: float = 0.5  # azimuth update weight for a new confident hit


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
    ) -> tuple[str, str]:
        """Returns (speaker, via) with via in {"voice", "direction", "none"}."""
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
        if azimuth_deg is None or not doa_reliable:
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
