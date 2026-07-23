"""Pure pick/place confirmation by re-detection (no ROS, offline-testable).
Count detections per class; a pick removes one of the target class, a place adds one."""


def count_by_class(names):
    """Count occurrences of each lowercased class name in `names`."""
    counts = {}
    for n in names:
        key = (n or "").lower()
        if key:
            counts[key] = counts.get(key, 0) + 1
    return counts


def picked_ok(before, after, target):
    """True if the target class count dropped (the picked object left the scene)."""
    t = (target or "").lower()
    return after.get(t, 0) < before.get(t, 0)


def placed_ok(before, after, target):
    """True if the target class count rose at the destination (object appeared)."""
    t = (target or "").lower()
    return after.get(t, 0) > before.get(t, 0)
