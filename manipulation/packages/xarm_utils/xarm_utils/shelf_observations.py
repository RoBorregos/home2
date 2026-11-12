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

    def add(self, levels: list[float]) -> list[float] | None:
        """Return stable median heights after a complete window of unique frames."""
        if any(not math.isfinite(level) for level in levels):
            raise ValueError("Shelf observations must be finite")
        self.samples.append(sorted(levels))
        if len(self.samples) < self.samples.maxlen:
            return None
        tracks: list[list[float]] = []
        for sample in self.samples:
            used = set()
            for level in sample:
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
                used.add(index)
        stable = sorted(
            median(track)
            for track in tracks
            if len(track) >= self.min_hits
            and max(track) - min(track) <= self.max_spread
        )
        return stable if len(stable) >= self.min_levels else None
