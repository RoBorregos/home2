"""Temporal consensus for shelf heights measured in a common robot frame."""

from collections import deque
import math
from statistics import median


class ShelfLevelConsensus:
    """Require repeated, stable evidence per level, allowing partial views."""

    def __init__(
        self,
        valid_frames: int = 12,
        max_spread: float = 0.015,
        min_levels: int = 2,
        support_fraction: float = 0.6,
        min_frames: int | None = None,
    ):
        if valid_frames < 3:
            raise ValueError("At least three frames are required for consensus")
        if not math.isfinite(max_spread) or max_spread <= 0:
            raise ValueError("Maximum frame spread must be positive")
        if min_levels < 1 or not 0.5 < support_fraction <= 1:
            raise ValueError("Invalid level count or support fraction")
        self.samples = deque(maxlen=valid_frames)
        self.max_spread = max_spread
        self.min_levels = min_levels
        self.min_hits = math.ceil(valid_frames * support_fraction)
        self.min_frames = min(6, valid_frames) if min_frames is None else min_frames
        if not 3 <= self.min_frames <= valid_frames:
            raise ValueError("Minimum frames must be between 3 and the window size")
        self.qualities = deque(maxlen=valid_frames)
        self.confirmation_mode = None

    def reset(self) -> None:
        """Forget both heights and quality when pose feedback becomes invalid."""
        self.samples.clear()
        self.qualities.clear()
        self.confirmation_mode = None

    def add(
        self, levels: list[float], evidence: list[dict] | None = None
    ) -> list[float] | None:
        """Confirm early only with repeated, well-distributed structural evidence.

        Height-only callers keep the full-window behavior. The caller must
        supply unique, timestamp-aligned frames; repeated values are not a clock.
        """
        if any(not math.isfinite(level) for level in levels):
            raise ValueError("Shelf observations must be finite")
        if evidence is not None and len(evidence) != len(levels):
            raise ValueError("Quality evidence must match the level count")
        order = sorted(range(len(levels)), key=levels.__getitem__)
        self.samples.append([levels[i] for i in order])
        self.qualities.append(
            [evidence[i] for i in order] if evidence is not None else None
        )
        if len(self.samples) < self.min_frames:
            return None
        tracks: list[list[float]] = []
        qualities: list[list[dict | None]] = []
        for sample, quality in zip(self.samples, self.qualities):
            used = set()
            for position, level in enumerate(sample):
                matches = [
                    i
                    for i, track in enumerate(tracks)
                    if i not in used and abs(level - median(track)) <= self.max_spread
                ]
                if matches:
                    index = min(matches, key=lambda i: abs(level - median(tracks[i])))
                    tracks[index].append(level)
                else:
                    index = len(tracks)
                    tracks.append([level])
                    qualities.append([])
                qualities[index].append(
                    quality[position] if quality is not None else None
                )
                used.add(index)
        if len(self.samples) < self.samples.maxlen:
            if any(q is None for q in self.qualities):
                return None
            ready = [
                i for i, t in enumerate(tracks) if self._fast_track(t, qualities[i])
            ]
            unresolved = any(
                i not in ready
                and len(track) >= 2
                and any(self._good_quality(q) for q in qualities[i])
                for i, track in enumerate(tracks)
            )
            if len(ready) >= self.min_levels and not unresolved:
                self.confirmation_mode = "quality"
                return sorted(median(tracks[i]) for i in ready)
            return None
        stable = sorted(
            median(track)
            for track in tracks
            if len(track) >= self.min_hits
            and max(track) - min(track) <= self.max_spread
        )
        if len(stable) >= self.min_levels:
            self.confirmation_mode = "full_window"
            return stable
        return None

    @staticmethod
    def _good_quality(evidence) -> bool:
        if not evidence or evidence.get("calibration_warning", True):
            return False
        values = [
            evidence.get(k, float("nan"))
            for k in (
                "quality_score",
                "width_coverage",
                "depth_coverage",
                "residual_p90_m",
            )
        ]
        return (
            all(math.isfinite(v) for v in values)
            and values[0] >= 0.8
            and values[1] >= 0.8
            and values[2] >= 0.65
            and values[3] <= 0.008
            and evidence.get("boundary_count", 0) == 3
        )

    def _fast_track(self, track, qualities) -> bool:
        count = len(self.samples)
        if (
            len(track) < math.ceil(count * 0.85)
            or max(track) - min(track) > self.max_spread / 2
        ):
            return False
        good = [q for q in qualities if self._good_quality(q)]
        if len(good) < math.ceil(count * 0.8):
            return False
        normals = []
        for q in good:
            normal = q.get("normal", [])
            if len(normal) != 3 or not all(math.isfinite(v) for v in normal):
                return False
            length = math.sqrt(sum(v * v for v in normal))
            if length < 1e-8:
                return False
            normals.append([v / length for v in normal])
        mean = [sum(n[i] for n in normals) for i in range(3)]
        length = math.sqrt(sum(v * v for v in mean))
        if length < 1e-8:
            return False
        return all(
            sum(a * b for a, b in zip(n, mean)) / length >= math.cos(math.radians(3))
            for n in normals
        )
