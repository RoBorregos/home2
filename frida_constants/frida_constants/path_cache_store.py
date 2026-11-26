"""Disk copy of navigation's named-pair path cache, readable while nav is down."""

import contextlib
import hashlib
import json
import math
import os
import tempfile
import time

# 2: per-map file with "map" and "saved_at" = when nav loaded the map it planned on
PATH_CACHE_VERSION = 2
# Files that define the planning map; editing any of them makes the cache stale
MAP_FILE_SUFFIXES = (".posegraph", ".data", ".yaml", ".pgm")


def path_cache_file(cache_dir, map_name):
    """One cache file per map, so switching maps keeps each map's pairs."""
    return os.path.join(cache_dir, f"{map_name}.json")


def path_cache_digest(*parts):
    """Stable identity of the map/areas the distances were planned on."""
    text = json.dumps(parts, sort_keys=True, default=str)
    return hashlib.sha256(text.encode()).hexdigest()


def map_files_changed_since(maps_dir, map_name, since):
    """Names of the map's files modified after ``since`` (epoch seconds)."""
    changed = []
    for suffix in MAP_FILE_SUFFIXES:
        name = f"{map_name}{suffix}"
        with contextlib.suppress(OSError):
            if os.path.getmtime(os.path.join(maps_dir, name)) > since:
                changed.append(name)
    return changed


def save_path_cache(path, digest, areas, cache, *, map_name=None, saved_at=None):
    """Atomically write {((loc, sub), (loc, sub)): meters} with its areas table."""
    data = {
        "version": PATH_CACHE_VERSION,
        "digest": digest,
        "map": map_name,
        "saved_at": time.time() if saved_at is None else saved_at,
        "areas": areas,
        "pairs": [[a[0], a[1], b[0], b[1], d] for (a, b), d in cache.items()],
    }
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix=".path_cache_")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(data, f)
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
    except BaseException:
        with contextlib.suppress(OSError):
            os.unlink(tmp)
        raise


def load_path_cache(path):
    """(digest, areas, pairs, saved_at, map) from save_path_cache, or None if
    missing or unreadable."""
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or data.get("version") != PATH_CACHE_VERSION:
        return None
    areas = data.get("areas")
    rows = data.get("pairs")
    saved_at = data.get("saved_at")
    if (
        not isinstance(areas, dict)
        or not isinstance(rows, list)
        or not isinstance(saved_at, (int, float))
        or isinstance(saved_at, bool)
        or not math.isfinite(saved_at)
    ):
        return None
    pairs = {}
    for row in rows:
        if (
            isinstance(row, list)
            and len(row) == 5
            and all(isinstance(v, str) for v in row[:4])
            and isinstance(row[4], (int, float))
            and not isinstance(row[4], bool)
            and math.isfinite(row[4])
            and row[4] >= 0.0
        ):
            pairs[((row[0], row[1]), (row[2], row[3]))] = float(row[4])
    return data.get("digest"), areas, pairs, float(saved_at), data.get("map")
