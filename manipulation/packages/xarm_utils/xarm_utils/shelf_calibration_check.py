#!/usr/bin/env python3
"""Validate the calibrated shelf geometry before running manipulation."""

import argparse
import sys

from xarm_utils.shelf_levels import (
    ShelfCalibrationError,
    load_shelf_calibration,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arena", type=int, default=1, help="Arena number (default: 1)"
    )
    parser.add_argument(
        "--file",
        help="Calibration JSON path. Defaults to FRIDA_SHELF_LEVELS_FILE or package config.",
    )
    args = parser.parse_args()

    try:
        calibration = load_shelf_calibration(
            args.arena,
            path=args.file,
        )
    except ShelfCalibrationError as exc:
        print(f"INVALID shelf calibration: {exc}", file=sys.stderr)
        return 1

    print(f"VALID shelf calibration: arena={args.arena}, frame={calibration.frame}")
    for level, (height, ceiling) in enumerate(
        zip(calibration.levels, calibration.ceiling_heights), start=1
    ):
        print(f"  level {level}: surface_z={height:.3f} m, ceiling_z={ceiling:.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
