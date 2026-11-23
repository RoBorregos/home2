#!/usr/bin/env python3
"""Hermetic test for fusing repeated hand detections (no ROS).
python3 task_manager/scripts/test/test_hand_point.py"""

import importlib.util
import os

_MOD = os.path.join(
    os.path.dirname(__file__),
    "..",
    "..",
    "task_manager",
    "utils",
    "hand_point.py",
)
_spec = importlib.util.spec_from_file_location("hand_point", _MOD)
hp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(hp)


def _close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test_agreeing_points_fuse_to_their_median():
    points = [(0.50, 0.00, 1.00), (0.51, 0.01, 1.01), (0.49, -0.01, 0.99)]
    assert _close(hp.fuse_points(points, 0.05), (0.50, 0.00, 1.00))


def test_an_outlier_is_dropped():
    # One keypoint jumped to the other wrist; the rest still agree.
    points = [(0.50, 0.0, 1.0), (0.51, 0.0, 1.0), (0.50, 0.01, 1.0), (0.50, 0.40, 1.0)]
    assert _close(hp.fuse_points(points, 0.05), (0.50, 0.0, 1.0))


def test_too_few_points_is_none():
    assert hp.fuse_points([(0.5, 0.0, 1.0), (0.5, 0.0, 1.0)], 0.05) is None
    assert hp.fuse_points([], 0.05) is None


def test_scattered_points_are_none():
    points = [(0.5, 0.0, 1.0), (0.6, 0.1, 1.0), (0.4, -0.1, 1.1), (0.7, 0.2, 0.9)]
    assert hp.fuse_points(points, 0.05) is None


def test_min_points_is_respected():
    points = [(0.5, 0.0, 1.0), (0.5, 0.0, 1.0), (0.9, 0.0, 1.0)]
    assert hp.fuse_points(points, 0.05, min_points=2) is not None
    assert hp.fuse_points(points, 0.05, min_points=3) is None


def test_returns_floats_from_any_number_type():
    assert hp.fuse_points([(1, 0, 1)] * 3, 0.05) == (1.0, 0.0, 1.0)


if __name__ == "__main__":
    test_agreeing_points_fuse_to_their_median()
    test_an_outlier_is_dropped()
    test_too_few_points_is_none()
    test_scattered_points_are_none()
    test_min_points_is_respected()
    test_returns_floats_from_any_number_type()
    print("test_hand_point: OK")
