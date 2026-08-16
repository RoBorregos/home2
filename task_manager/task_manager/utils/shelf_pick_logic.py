"""Pure decision logic for per-level shelf picking.

Mirrors the level-assignment rule used by storing_groceries so the shelf pick can
detect the target at the level whose viewing pose actually frames it. No ROS imports,
so it is unit-testable offline.
"""

import json
import os
from pathlib import Path

# Calibrated shelf level heights live in a JSON file so they can be re-measured at
# competition without editing code. Override with FRIDA_SHELF_LEVELS_FILE.
SHELF_LEVELS_FILE = Path(
    os.environ.get("FRIDA_SHELF_LEVELS_FILE", Path.home() / "frida_shelf_levels.json")
)

# Defaults match storing_groceries_manager (shelf_level_threshold /
# shelf_level_down_threshold). A detection counts as "on this level" if it sits just
# below or up to UP_THRESHOLD above the level height.
LEVEL_UP_THRESHOLD = 0.20
LEVEL_DOWN_THRESHOLD = 0.05


def height_matches_level(
    height: float,
    level_height: float,
    up_threshold: float = LEVEL_UP_THRESHOLD,
    down_threshold: float = LEVEL_DOWN_THRESHOLD,
) -> bool:
    """True if a detection at `height` belongs to the shelf at `level_height`."""
    distance = height - level_height
    if distance < 0:
        return abs(distance) < down_threshold
    return distance < up_threshold


def _name_matches(target: str, name: str) -> bool:
    t = (target or "").lower()
    n = (name or "").lower()
    if not t or not n:
        return False
    return t == n or t in n or n in t


def find_target_on_level(
    candidates,
    target,
    level_height,
    up_threshold: float = LEVEL_UP_THRESHOLD,
    down_threshold: float = LEVEL_DOWN_THRESHOLD,
):
    """Return the first candidate matching `target` framed at `level_height`, else None.

    candidates: iterable of (name, height) pairs. A candidate whose height is None
    (e.g. its TF lookup failed) is skipped: without a usable height a name-only match
    would anchor the target to the wrong (first-scanned) level.
    """
    for name, height in candidates:
        if not _name_matches(target, name):
            continue
        if height is not None and height_matches_level(
            height, level_height, up_threshold, down_threshold
        ):
            return (name, height)
    return None


def levels_from_sorted_heights(heights):
    """Map measured surface heights to the {1,2,3} level dict (1 = lowest)."""
    ordered = sorted(float(h) for h in heights)
    return {i + 1: h for i, h in enumerate(ordered)}


def load_shelf_levels(default, path=SHELF_LEVELS_FILE):
    """Return the calibrated {level: height} dict from the JSON file, else `default`.
    Never raises; falls back to default on read/parse error or level-count mismatch."""
    try:
        data = json.loads(Path(path).read_text())
        levels = {int(k): float(v) for k, v in data["levels"].items()}
        if len(levels) == len(default):
            return levels
    except Exception:
        pass
    return default


def save_shelf_levels(levels, path=SHELF_LEVELS_FILE):
    """Write the {level: height} dict (base_link Z) to the JSON file; return the path."""
    payload = {"frame": "base_link", "levels": {str(k): float(v) for k, v in levels.items()}}
    p = Path(path)
    p.write_text(json.dumps(payload, indent=2))
    return p
