#!/usr/bin/env python3
"""Remove small isolated obstacle specks from a saved map (.pgm) in place.

Mapping leaves specks where people or moved chairs were seen. Nav2 blocks the
robot's inscribed radius around every one of them, so a cluster of specks can
seal a corridor that is really open (lab_20_nov: bedroom unreachable). Real
small obstacles (chair legs) are still seen live by the lidar.

Only the .pgm (Nav2 static layer) changes; slam_toolbox localizes with the
.posegraph, which is untouched. The original is kept as <map>.raw.pgm.

Usage: clean_map_noise.py <map.pgm> [--max-px 4]
"""

import argparse
import shutil
from collections import deque
from pathlib import Path

import numpy as np

OCCUPIED_MAX = 89  # trinary map, occupied_thresh 0.65 -> pixel <= 255 * 0.35
FREE = 254


def read_pgm(path: Path) -> tuple[bytes, np.ndarray]:
    data = path.read_bytes()
    fields, pos = [], 0
    while len(fields) < 4:  # magic, width, height, maxval (skip comments)
        while data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end : end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    pos += 1  # single whitespace before the pixels
    if fields[0] != b"P5":
        raise SystemExit(f"{path}: only binary P5 maps are supported")
    width, height = int(fields[1]), int(fields[2])
    pixels = np.frombuffer(data[pos : pos + width * height], dtype=np.uint8)
    return data[:pos], pixels.reshape(height, width).copy()


def small_specks(occupied: np.ndarray, max_px: int) -> list[list[tuple[int, int]]]:
    """8-connected groups of occupied pixels with at most max_px pixels."""
    seen = np.zeros_like(occupied, dtype=bool)
    height, width = occupied.shape
    specks = []
    for r, c in zip(*np.nonzero(occupied)):
        if seen[r, c]:
            continue
        group, queue = [], deque([(r, c)])
        seen[r, c] = True
        while queue:
            y, x = queue.popleft()
            group.append((y, x))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < height and 0 <= nx < width and occupied[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        queue.append((ny, nx))
        if len(group) <= max_px:
            specks.append(group)
    return specks


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("pgm", type=Path)
    parser.add_argument("--max-px", type=int, default=4, help="largest speck removed (pixels)")
    args = parser.parse_args()

    raw = args.pgm.with_suffix(".raw.pgm")
    if not raw.exists():
        shutil.copy2(args.pgm, raw)
    header, pixels = read_pgm(raw)  # always clean from the original

    specks = small_specks(pixels <= OCCUPIED_MAX, args.max_px)
    for group in specks:
        for y, x in group:
            pixels[y, x] = FREE
    args.pgm.write_bytes(header + pixels.tobytes())
    print(f"{args.pgm}: removed {len(specks)} specks (<= {args.max_px} px); original in {raw}")


if __name__ == "__main__":
    main()
