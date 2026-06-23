"""Pure decision logic for per-level shelf picking (no ROS, unit-testable offline).
Mirrors the level-assignment rule used by storing_groceries."""

# Defaults match storing_groceries. A detection is "on this level" if it sits just
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
    """First candidate matching target framed at level_height, else None.
    candidates: (name, height) pairs; height None skips the height check."""
    for name, height in candidates:
        if not _name_matches(target, name):
            continue
        if height is None or height_matches_level(
            height, level_height, up_threshold, down_threshold
        ):
            return (name, height)
    return None
