#!/usr/bin/env python3
"""Burn the occupied cells of one map_server grid into another (#1318).

Use: ZED grid from RTAB-Map (zed_grid.launch.py) on top of the slam_toolbox
grid, so furniture the lidars miss ends up in the static /map. Only OCCUPIED
overlay cells are copied; the base's free/unknown/occupied cells are otherwise
untouched, and the slam_toolbox .posegraph (localization) is not involved.

    merge_grids.py <base.yaml> <overlay.yaml> <out_basename>
    -> <out_basename>.pgm + <out_basename>.yaml (base metadata, new image)

Both grids must share the same frame (zed_grid.launch.py guarantees it).
    merge_grids.py --test   # self-check
"""

import argparse
import math
import os
import sys
import tempfile

import yaml


def load(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    if float(meta['origin'][2]) != 0.0:
        sys.exit(f'{yaml_path}: rotated origin not supported')
    img = meta['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img)
    with open(img, 'rb') as f:
        data = f.read()
    # P5 header: magic, width, height, maxval; '#' comments allowed (map_saver writes one).
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b'P5' or int(tokens[3]) > 255:
        sys.exit(f'{img}: only 8-bit binary PGM (P5) supported')
    w, h = int(tokens[1]), int(tokens[2])
    pixels = bytearray(data[pos + 1:pos + 1 + w * h])
    return meta, w, h, pixels


def is_occupied(p, meta):
    occ = p / 255.0 if meta.get('negate', 0) else (255 - p) / 255.0
    return occ > float(meta.get('occupied_thresh', 0.65))


def merge(base, overlay):
    """OR overlay's occupied cells into base (in place). Returns (merged, dropped)."""
    bmeta, bw, bh, bpix = base
    ometa, ow, oh, opix = overlay
    bres, ores = float(bmeta['resolution']), float(ometa['resolution'])
    bx, by = map(float, bmeta['origin'][:2])
    ox, oy = map(float, ometa['origin'][:2])
    occ_val = 255 if bmeta.get('negate', 0) else 0
    merged = dropped = 0
    for row in range(oh):          # PGM row 0 is the TOP of the map
        for col in range(ow):
            if not is_occupied(opix[row * ow + col], ometa):
                continue
            x = ox + (col + 0.5) * ores
            y = oy + (oh - row - 0.5) * ores
            bc = math.floor((x - bx) / bres)
            br = bh - 1 - math.floor((y - by) / bres)
            # ponytail: overlay cells outside the base are dropped, not grown into a bigger map.
            if not (0 <= bc < bw and 0 <= br < bh):
                dropped += 1
                continue
            if bpix[br * bw + bc] != occ_val:
                bpix[br * bw + bc] = occ_val
                merged += 1
    return merged, dropped


def save(grid, out_base):
    meta, w, h, pix = grid
    with open(out_base + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode() + bytes(pix))
    meta = {**meta, 'image': os.path.basename(out_base) + '.pgm'}
    with open(out_base + '.yaml', 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)


def _write_grid(d, name, w, h, pix, origin, res=0.05):
    with open(os.path.join(d, name + '.pgm'), 'wb') as f:
        f.write(f'P5\n# CREATOR: test\n{w} {h}\n255\n'.encode() + bytes(pix))
    path = os.path.join(d, name + '.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump({'image': name + '.pgm', 'resolution': res, 'origin': origin + [0.0],
                        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}, f)
    return path


def self_check():
    with tempfile.TemporaryDirectory() as d:
        base = [254] * 16
        base[15] = 0                                    # bottom-right already occupied
        # 2x2 overlay shifted one cell in x and y: occupied, free / unknown, free.
        bpath = _write_grid(d, 'base', 4, 4, base, [0.0, 0.0])
        opath = _write_grid(d, 'over', 2, 2, [0, 254, 205, 254], [0.05, 0.05])
        out = os.path.join(d, 'out')
        b = load(bpath)
        assert merge(b, load(opath)) == (1, 0)
        save(b, out)
        _, w, h, pix = load(out + '.yaml')
        # Overlay top-left (x=0.075, y=0.125) -> base col 1, 2 rows up from the bottom -> PGM row 1.
        expected = [254] * 16
        expected[1 * 4 + 1] = 0
        expected[15] = 0                                # free overlay never clears the base
        assert (w, h, list(pix)) == (4, 4, expected), list(pix)
        # Outside the base -> dropped.
        far = load(_write_grid(d, 'far', 1, 1, [0], [10.0, 10.0]))
        assert merge(load(bpath), far) == (0, 1)
    print('merge_grids self-check OK')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('base', nargs='?')
    ap.add_argument('overlay', nargs='?')
    ap.add_argument('out', nargs='?')
    ap.add_argument('--test', action='store_true')
    a = ap.parse_args()
    if a.test:
        return self_check()
    if not (a.base and a.overlay and a.out):
        ap.error('base, overlay and out are required')
    base = load(a.base)
    merged, dropped = merge(base, load(a.overlay))
    save(base, a.out)
    print(f'{merged} cells added, {dropped} overlay cells outside the base dropped -> {a.out}.yaml')


if __name__ == '__main__':
    main()
