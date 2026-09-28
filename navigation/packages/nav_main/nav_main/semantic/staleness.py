"""Per-viewpoint scan bookkeeping: when did the robot last look at each surface.

This is navigation's own state, not vision's: it records that the base stood at a
viewpoint facing its furniture, which is what makes a revisit route possible even
before the object map exists. Persisted as JSON so a restart mid-task does not
forget the whole arena.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import time

from .surfaces import Viewpoint

SCHEMA_VERSION = 1


def angle_difference(a: float, b: float) -> float:
    """Smallest absolute difference between two angles, in radians."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class ScanLog:
    """`area/name` -> timestamp of the last scan."""

    def __init__(self, map_name: str = "", path: str | None = None):
        self.map_name = map_name
        self.path = path
        self.scans: dict[str, float] = {}
        self._dirty = False

    # ------------------------------------------------------------------ updates

    def mark(self, key: str, stamp: float) -> None:
        self.scans[key] = stamp
        self._dirty = True

    def mark_if_at(
        self,
        viewpoints: list[Viewpoint],
        x: float,
        y: float,
        yaw: float,
        stamp: float,
        radius: float,
        yaw_tolerance_rad: float,
        closest_only: bool = True,
    ) -> list[str]:
        """Mark the viewpoint the robot is currently standing at and facing.

        A viewpoint counts as scanned when the base is within `radius` of the
        tagged pose and its heading is within `yaw_tolerance_rad` of the tagged
        heading — the camera rides on the gripper, so pose *and* heading matter.

        `closest_only` (the default) marks just the nearest match. Tagged poses in
        a kitchen sit well under a metre apart, so marking every pose in range
        would claim surfaces the arm never stared at and then skip them on the
        next revisit round. One stop, one surface. Returns the keys marked.
        """
        candidates = [
            vp
            for vp in viewpoints
            if vp.distance_to(x, y) <= radius
            and angle_difference(yaw, vp.yaw) <= yaw_tolerance_rad
        ]
        if not candidates:
            return []
        if closest_only:
            candidates = [min(candidates, key=lambda vp: vp.distance_to(x, y))]
        for vp in candidates:
            self.mark(vp.key, stamp)
        return [vp.key for vp in candidates]

    def age(self, key: str, now: float, never: float = float("inf")) -> float:
        stamp = self.scans.get(key)
        return never if stamp is None else now - stamp

    def clear(self) -> None:
        self.scans.clear()
        self._dirty = True

    # -------------------------------------------------------------- persistence

    @property
    def dirty(self) -> bool:
        return self._dirty

    def to_dict(self, now: float, wall_now: float | None = None) -> dict:
        """Ages relative to the save, plus the wall clock at save time.

        Ages (not absolute stamps) because a ROS clock restarts at 0 on a bag
        replay, so stored stamps would come back from the future. The wall clock
        is what makes a restart honest: on reload the gap since the save is added
        back, otherwise everything would look freshly scanned.
        """
        return {
            "version": SCHEMA_VERSION,
            "map_name": self.map_name,
            "saved_at_wall": time.time() if wall_now is None else wall_now,
            "ages": {key: now - stamp for key, stamp in self.scans.items()},
        }

    def load_dict(self, data: dict, now: float, wall_now: float | None = None) -> tuple[int, str]:
        """Returns ``(restored_count, reason_when_skipped)``."""
        if not isinstance(data, dict):
            return 0, "snapshot is not an object"
        if data.get("version") != SCHEMA_VERSION:
            return 0, f"version mismatch: {data.get('version')!r}"
        if self.map_name and data.get("map_name") not in (self.map_name, ""):
            return 0, f"snapshot belongs to map {data.get('map_name')!r}"
        ages = data.get("ages") or {}
        if not isinstance(ages, dict):
            return 0, "'ages' is not an object"

        # Add back the time the robot spent switched off, so a surface scanned
        # ten minutes before the restart reads as ten minutes stale, not fresh.
        try:
            saved_at_wall = float(data.get("saved_at_wall"))
            elapsed = max(0.0, (wall_now if wall_now is not None else time.time()) - saved_at_wall)
        except (TypeError, ValueError):
            elapsed = 0.0

        restored = 0
        for key, age in ages.items():
            try:
                age = float(age)
            except (TypeError, ValueError):
                continue
            if age < 0:
                continue
            self.scans[key] = now - (age + elapsed)
            restored += 1
        self._dirty = False
        return restored, ""

    def save(self, now: float, path: str | None = None, wall_now: float | None = None) -> str:
        """Atomic write (tmp + fsync + replace): this file is read at startup
        during a competition run, so a half-written JSON is not acceptable."""
        target = path or self.path
        if not target:
            raise ValueError("no path configured for the scan log")
        os.makedirs(os.path.dirname(target), exist_ok=True)
        payload = json.dumps(self.to_dict(now, wall_now), indent=2)
        directory = os.path.dirname(target)
        with tempfile.NamedTemporaryFile(
            "w", dir=directory, prefix=".staleness-", suffix=".tmp", delete=False
        ) as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
            tmp = handle.name
        os.replace(tmp, target)
        self._dirty = False
        return target

    def load(self, now: float, path: str | None = None, wall_now: float | None = None) -> tuple[int, str]:
        target = path or self.path
        if not target or not os.path.exists(target):
            return 0, "no snapshot file"
        try:
            with open(target, "r") as handle:
                return self.load_dict(json.load(handle), now, wall_now)
        except (OSError, ValueError) as exc:
            return 0, f"unreadable snapshot: {exc}"
