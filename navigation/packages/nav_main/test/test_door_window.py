import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from nav_central import door_window_median  # noqa: E402

inf, nan = float("inf"), float("nan")


def test_window_median():
    assert door_window_median([9, 1.0, 2.0, 9], 1, 2, 0.1, 12.0) == 1.5


def test_wrap_around_window():
    assert door_window_median([1.0, 9, 9, 3.0], 3, 0, 0.1, 12.0) == 2.0


def test_inf_is_far_nan_and_too_close_ignored():
    assert door_window_median([inf, nan, 0.01], 0, 2, 0.1, 12.0) == 12.0


def test_no_valid_beams():
    assert door_window_median([nan, nan], 0, 1, 0.1, 12.0) is None


def test_closed_door_with_empty_bins():
    # Merged scan in front of a closed door: a few empty bins must not read as open
    assert door_window_median([0.5] * 10 + [inf] * 3, 0, 12, 0.1, 12.0) == 0.5
