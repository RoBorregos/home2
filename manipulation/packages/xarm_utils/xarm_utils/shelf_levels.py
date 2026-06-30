"""Shelf level heights, stored in config/shelf_levels.json so they can be edited
on site without recompiling.

Read the heights for an arena with ``get_shelf_levels(arena)``; write them with
``ros2 run xarm_utils shelf_height_calibrator.py --arena N``.
"""

import json
import os
from pathlib import Path

SHELF_SCAN_TOLERANCE = 0.1

# Fallback shelf level-to-ceiling (compartment) height, in meters. Only used if
# the JSON levels file cannot be read; the live value comes from
# config/shelf_levels.json ("compartment_height").
COMPARTMENT_HEIGHT_FALLBACK = 0.34

# Fallback shelf level surface heights in base_link Z, per competition arena.
# Only used if the JSON levels file cannot be read; the live values come from
# config/shelf_levels.json.
SHELF_LEVELS_BY_ARENA_FALLBACK = {
    1: [0.599, 0.946, 1.298],
    2: [0.599, 0.946, 1.298],
    3: [0.599, 0.946, 1.298],
}


def levels_file():
    """Resolve the shelf levels path: FRIDA_SHELF_LEVELS_FILE env, else the
    xarm_utils package share config."""
    env = os.environ.get("FRIDA_SHELF_LEVELS_FILE")
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("xarm_utils"))
        return share / "config" / "shelf_levels.json"
    except Exception:
        return Path.home() / "frida_shelf_levels.json"


def get_shelf_levels(arena: int) -> list:
    """Return the 3 shelf heights for the selected arena, read live from the JSON
    levels file (no recompile needed). Falls back to arena 1, then to the
    hardcoded defaults, if the file or arena entry is missing."""
    arena = int(arena)
    try:
        data = json.loads(levels_file().read_text())
        arenas = data.get("arenas", {})
        levels = arenas.get(str(arena)) or arenas.get("1")
        if levels:
            return [float(h) for h in levels]
    except Exception:
        pass
    return SHELF_LEVELS_BY_ARENA_FALLBACK.get(arena, SHELF_LEVELS_BY_ARENA_FALLBACK[1])


def get_compartment_height() -> float:
    """Return the shelf level-to-ceiling (compartment) height, read live from the
    JSON levels file (no recompile needed). Falls back to the hardcoded default if
    the file or the "compartment_height" entry is missing."""
    try:
        data = json.loads(levels_file().read_text())
        height = data.get("compartment_height")
        if height is not None:
            return float(height)
    except Exception:
        pass
    return COMPARTMENT_HEIGHT_FALLBACK
