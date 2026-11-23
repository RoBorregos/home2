"""Pure fusion of repeated hand detections into one point (no ROS, offline-testable).
The median rejects a single bad keypoint; points farther than max_spread from it are dropped."""

from statistics import median


def _median_point(points):
    return tuple(median(p[axis] for p in points) for axis in range(3))


def _distance(a, b):
    return sum((a[axis] - b[axis]) ** 2 for axis in range(3)) ** 0.5


def fuse_points(points, max_spread, min_points=3):
    """Median (x, y, z) of the points that agree, or None if fewer than min_points do.

    points: iterable of (x, y, z); a point agrees when it lies within max_spread
    (metres) of the median of all the points.
    """
    points = [tuple(float(v) for v in p) for p in points]
    if len(points) < min_points:
        return None
    centre = _median_point(points)
    agreeing = [p for p in points if _distance(p, centre) <= max_spread]
    if len(agreeing) < min_points:
        return None
    return _median_point(agreeing)
