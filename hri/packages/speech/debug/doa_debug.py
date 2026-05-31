#!/usr/bin/env python3
"""
DOA (Direction of Arrival) visual debugger for the ReSpeaker mic array.

Run directly — no ROS needed. Press Ctrl+C to exit.

Usage:
    python3 doa_debug.py
"""

import math
import os
import sys
import time

import usb.core

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from speech.tuning import Tuning

REFRESH_HZ = 10
COMPASS_RADIUS = 8
DOA_FUSION_WEIGHT = 0.3  # Must match follow_face_node.py


def find_device():
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if dev is None:
        print("ERROR: ReSpeaker not found. Check USB connection.")
        sys.exit(1)
    return dev


def draw_compass(angle_deg: int) -> str:
    """ASCII compass showing the DOA angle."""
    r = COMPASS_RADIUS
    grid = [[" "] * (2 * r + 1) for _ in range(2 * r + 1)]

    # Draw circle border
    for deg in range(0, 360, 3):
        rad = math.radians(deg)
        x = int(round(r + (r - 1) * math.sin(rad)))
        y = int(round(r - (r - 1) * math.cos(rad)))
        grid[y][x] = "·"

    # Cardinal labels
    grid[0][r] = "0"
    grid[r][2 * r] = "9"  # 90°
    grid[2 * r][r] = "1"  # 180°  (just the first digit)
    grid[r][0] = "2"  # 270°

    # Direction arrow (from center toward source)
    rad = math.radians(angle_deg)
    for step in range(1, r):
        x = int(round(r + step * math.sin(rad)))
        y = int(round(r - step * math.cos(rad)))
        if 0 <= x <= 2 * r and 0 <= y <= 2 * r:
            grid[y][x] = "█" if step == r - 1 else "▪"

    # Center
    grid[r][r] = "◎"

    rows = ["".join(row) for row in grid]
    return "\n".join(rows)


class MovingAverage:
    def __init__(self, n=10):
        self._buf = [0] * n
        self._n = n
        self._sum = 0
        self._idx = 0
        self._size = 0

    def next(self, val):
        if self._size < self._n:
            self._size += 1
        self._sum -= self._buf[self._idx]
        self._buf[self._idx] = val
        self._sum += val
        self._idx = (self._idx + 1) % self._n
        return self._sum / self._size


def angle_to_x_error(angle_deg: int) -> float:
    """Same conversion used in follow_face_node._doa_x_error()."""
    return math.sin(math.radians(angle_deg))


def main():
    dev = find_device()
    tuning = Tuning(dev)
    avg = MovingAverage(10)

    print("ReSpeaker DOA debugger — Ctrl+C to quit\n")

    try:
        while True:
            raw = tuning.direction
            smoothed = int(avg.next(raw))
            is_voice = bool(tuning.is_voice)
            x_error = angle_to_x_error(smoothed)

            os.system("clear")

            compass = draw_compass(smoothed)

            voice_str = "🔊 SPEAKING" if is_voice else "   silent  "
            bar_len = 20
            bar_val = int((x_error + 1) / 2 * bar_len)
            bar = "[" + "█" * bar_val + "·" * (bar_len - bar_val) + "]"

            print("═" * 45)
            print("  ReSpeaker DOA Debug")
            print("═" * 45)
            print(f"\n  Voice:    {voice_str}")
            print(f"  Raw DOA:  {raw:>4}°")
            print(f"  Smoothed: {smoothed:>4}°\n")
            print(compass)
            print(f"\n  x_error (sin):  {x_error:+.3f}")
            print(f"  L {bar} R")
            print(f"\n  With vision (DOA_FUSION_WEIGHT={DOA_FUSION_WEIGHT}):")
            print(f"    fused_x = 0.7 * vision_x + 0.3 * {x_error:+.3f}")
            print("\n  (0° = front, 90° = right, 270° = left)")
            print("═" * 45)

            time.sleep(1.0 / REFRESH_HZ)

    except KeyboardInterrupt:
        print("\nBye.")


if __name__ == "__main__":
    main()
