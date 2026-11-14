"""Shelf level heights, stored in config/shelf_levels.json so they can be edited
on site without recompiling.

Read the heights for an arena with ``get_shelf_levels(arena)``; write them with
``ros2 run xarm_utils shelf_height_calibrator.py --arena N``.
"""

import json
import math
import os
import stat
import tempfile
from dataclasses import dataclass
from pathlib import Path

SHELF_SCAN_TOLERANCE = 0.1
MIN_SHELF_LEVELS = 2
MAX_SHELF_LEVELS = 8
DEFAULT_MIN_LEVEL_SEPARATION = 0.10

# Fallback shelf level-to-ceiling (compartment) height, in meters. Only used if
# the JSON levels file cannot be read; the live value comes from
# config/shelf_levels.json ("compartment_height"). Must mirror that file: a
# too-large fallback puts the ceiling guard slab ABOVE the real ceiling,
# disabling the protection.
COMPARTMENT_HEIGHT_FALLBACK = 0.26

# Fallback shelf level surface heights in base_link Z, per competition arena.
# Only used if the JSON levels file cannot be read; the live values come from
# config/shelf_levels.json. Must mirror that file (4 REAL levels, 1 = lowest)
# so shelf level NUMBERING does not shift when falling back — the announced
# level number scores points.
SHELF_LEVELS_BY_ARENA_FALLBACK = {
    1: [0.095, 0.39, 0.70, 1.05],
    2: [0.095, 0.39, 0.68, 1.05],
    3: [0.095, 0.39, 0.68, 1.05],
}


class ShelfCalibrationError(ValueError):
    """Raised when shelf calibration data cannot be used safely."""


@dataclass(frozen=True)
class ShelfCalibration:
    """Validated shelf geometry for one arena."""

    frame: str
    levels: tuple[float, ...]
    compartment_height: float

    @property
    def ceiling_heights(self) -> tuple[float, ...]:
        """Return the estimated ceiling height above each shelf level."""
        return tuple(level + self.compartment_height for level in self.levels)


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


def write_json_atomic(path: Path, data: dict, validate=None) -> None:
    """Replace a JSON file only after serialization and optional validation."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            "w", dir=path.parent, prefix=f".{path.name}.", suffix=".tmp", delete=False
        ) as stream:
            temporary = Path(stream.name)
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write("\n")
        if validate is not None:
            validate(temporary)
        mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o644
        temporary.chmod(mode)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def save_detected_levels(
    arena: int,
    levels: list[float],
    path: Path,
    *,
    min_level_separation: float = DEFAULT_MIN_LEVEL_SEPARATION,
) -> None:
    """Validate the proposed arena calibration before replacing the old file."""
    path = Path(path)
    data = json.loads(path.read_text()) if path.exists() else {}
    if data.get("frame", "base_link") != "base_link":
        raise ShelfCalibrationError("Existing calibration is not in base_link")
    data["frame"] = "base_link"
    data.setdefault("compartment_height", 0.26)
    data.setdefault("arenas", {})[str(arena)] = [
        round(float(level), 4) for level in levels
    ]
    write_json_atomic(
        path,
        data,
        lambda proposed: load_shelf_calibration(
            arena, proposed, min_level_separation=min_level_separation
        ),
    )


def load_shelf_calibration(
    arena: int,
    path: Path | None = None,
    expected_levels: int | None = None,
    min_levels: int = MIN_SHELF_LEVELS,
    max_levels: int = MAX_SHELF_LEVELS,
    min_level_separation: float = DEFAULT_MIN_LEVEL_SEPARATION,
) -> ShelfCalibration:
    """Load and validate one arena calibration.

    This is the strict API for startup checks and safety-critical consumers.
    ``get_shelf_levels`` remains backward compatible and keeps its fallback
    behavior for legacy callers.
    """
    calibration_path = Path(path) if path is not None else levels_file()
    try:
        data = json.loads(calibration_path.read_text())
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ShelfCalibrationError(
            f"Cannot read calibration file {calibration_path}: {exc}"
        ) from exc

    frame = data.get("frame")
    if frame != "base_link":
        raise ShelfCalibrationError(
            f"Calibration frame must be 'base_link', got {frame!r}"
        )

    try:
        compartment_height = float(data["compartment_height"])
        raw_levels = data["arenas"][str(int(arena))]
        levels = tuple(float(level) for level in raw_levels)
    except (KeyError, TypeError, ValueError) as exc:
        raise ShelfCalibrationError(
            f"Missing or invalid calibration for arena {arena}: {exc}"
        ) from exc

    values = (*levels, compartment_height)
    if not all(math.isfinite(value) for value in values):
        raise ShelfCalibrationError("Calibration values must be finite numbers")
    if expected_levels is not None and len(levels) != expected_levels:
        raise ShelfCalibrationError(
            f"Arena {arena} must contain {expected_levels} levels, got {len(levels)}"
        )
    if not min_levels <= len(levels) <= max_levels:
        raise ShelfCalibrationError(
            f"Arena {arena} must contain between {min_levels} and {max_levels} levels, "
            f"got {len(levels)}"
        )
    if compartment_height <= 0:
        raise ShelfCalibrationError(
            f"Compartment height must be positive, got {compartment_height}"
        )
    if any(current <= previous for previous, current in zip(levels, levels[1:])):
        raise ShelfCalibrationError("Shelf levels must be strictly increasing")
    if min_level_separation <= 0:
        raise ShelfCalibrationError("Minimum level separation must be positive")
    if any(
        current - previous < min_level_separation
        for previous, current in zip(levels, levels[1:])
    ):
        raise ShelfCalibrationError(
            f"Shelf levels must be at least {min_level_separation:.2f} m apart"
        )

    return ShelfCalibration(
        frame=frame,
        levels=levels,
        compartment_height=compartment_height,
    )


def get_shelf_levels(arena: int) -> list:
    """Return the shelf heights for the selected arena, read live from the JSON
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
