#!/usr/bin/env python3
"""Run inside the navigation container: python3 test/test_door_window.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from nav_central import door_window_avg  # noqa: E402

inf, nan = float("inf"), float("nan")
assert door_window_avg([9, 1.0, 2.0, 9], 1, 2, 0.1, 12.0) == 1.5
assert door_window_avg([1.0, 9, 9, 3.0], 3, 0, 0.1, 12.0) == 2.0  # wrap-around window
assert door_window_avg([inf, nan, 0.01], 0, 2, 0.1, 12.0) == 12.0  # inf -> far, nan/too-close ignored
assert door_window_avg([nan, nan], 0, 1, 0.1, 12.0) is None
print("ok")
