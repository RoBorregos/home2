"""Disk copy of navigation's named-pair path cache, readable while nav is down."""

import contextlib
import hashlib
import json
import math
import os
import tempfile
import time

PATH_CACHE_VERSION = 1


def path_cache_digest(*parts):
    """Stable identity of the map/areas the distances were planned on."""
    text = json.dumps(parts, sort_keys=True, default=str)
    return hashlib.sha256(text.encode()).hexdigest()


def save_path_cache(path, digest, areas, cache):
    """Atomically write {((loc, sub), (loc, sub)): meters} with its areas table."""
    data = {
        "version": PATH_CACHE_VERSION,
        "digest": digest,
        "saved_at": time.time(),
        "areas": areas,
        "pairs": [[a[0], a[1], b[0], b[1], d] for (a, b), d in cache.items()],
    }
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path) or ".", prefix=".path_cache_")
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
    """(digest, areas, pairs) from save_path_cache, or None if missing or unreadable."""
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or data.get("version") != PATH_CACHE_VERSION:
        return None
    areas = data.get("areas")
    rows = data.get("pairs")
    if not isinstance(areas, dict) or not isinstance(rows, list):
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
    return data.get("digest"), areas, pairs
