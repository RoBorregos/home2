#!/usr/bin/env python3
"""Renderiza la captura de la ZED (zed_capture.py) para validar el ROI (#944).

Lee rgb.npy / depth.npy de un dir y produce PNGs con una REJILLA en coordenadas
normalizadas [0..1] (las que usa `manual_polygon` de la ZED) para:
  - localizar el gripper en el RGB y leer sus coordenadas normalizadas;
  - ver en el depth qué región tiene profundidad válida (lo recortado por el ROI
    queda en negro);
  - opcional: dibujar un polígono candidato encima para previsualizar el ROI.

Uso:
  python3 render_zed_capture.py --dir /tmp/zed_capture_local
  python3 render_zed_capture.py --dir DIR --poly '[[0.0,0.38],[1.0,0.38],[1.0,1.0],[0.0,1.0]]'

NOTA ROI ZED: el manual_polygon define la región a CONSERVAR (procesar); el resto se
descarta. Coordenadas normalizadas (x,y) con origen arriba-izquierda, y hacia abajo.
"""
import os
import sys
import json
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly


def read_meta(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def add_grid(ax, h, w, title):
    ax.set_title(title, fontsize=10)
    for frac in [i / 10 for i in range(1, 10)]:
        ax.axvline(frac * w, color="cyan", lw=0.5, alpha=0.6)
        ax.axhline(frac * h, color="cyan", lw=0.5, alpha=0.6)
    # etiquetas normalizadas
    ax.set_xticks([f * w for f in [0, .2, .4, .6, .8, 1.0]])
    ax.set_xticklabels([f"{f:.1f}" for f in [0, .2, .4, .6, .8, 1.0]], fontsize=8)
    ax.set_yticks([f * h for f in [0, .2, .4, .6, .8, 1.0]])
    ax.set_yticklabels([f"{f:.1f}" for f in [0, .2, .4, .6, .8, 1.0]], fontsize=8)


def draw_poly(ax, poly, h, w):
    if not poly:
        return
    pts = [(x * w, y * h) for x, y in poly]
    ax.add_patch(MplPoly(pts, closed=True, fill=True, facecolor="lime",
                         edgecolor="lime", alpha=0.25, lw=2))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dir", default="/tmp/zed_capture_local")
    p.add_argument("--poly", default="", help="JSON: [[x,y],...] normalizado, región a CONSERVAR")
    a = p.parse_args()
    d = a.dir
    poly = json.loads(a.poly) if a.poly else None

    rgb_meta = read_meta(f"{d}/rgb_meta.txt")
    depth_meta = read_meta(f"{d}/depth_meta.txt")
    cloud_meta = read_meta(f"{d}/cloud_meta.txt")
    print("RGB  :", rgb_meta)
    print("DEPTH:", depth_meta)
    print("CLOUD:", cloud_meta)

    rgb = np.load(f"{d}/rgb.npy") if os.path.exists(f"{d}/rgb.npy") else None
    depth = np.load(f"{d}/depth.npy") if os.path.exists(f"{d}/depth.npy") else None

    # encoding BGR? -> voltear canales para mostrar
    if rgb is not None and "bgr" in rgb_meta.lower():
        rgb = rgb[..., ::-1]
    if rgb is not None and rgb.shape[-1] == 4:
        rgb = rgb[..., :3]

    n = (rgb is not None) + (depth is not None)
    fig, axes = plt.subplots(1, max(n, 1), figsize=(7 * max(n, 1), 6))
    if n == 1:
        axes = [axes]
    i = 0
    if rgb is not None:
        h, w = rgb.shape[:2]
        axes[i].imshow(rgb)
        add_grid(axes[i], h, w, f"RGB {w}x{h} — localiza el gripper (coords normalizadas)")
        draw_poly(axes[i], poly, h, w)
        i += 1
    if depth is not None:
        h, w = depth.shape[:2]
        dd = depth.copy()
        valid = np.isfinite(dd) & (dd > 0)
        vis = np.zeros_like(dd)
        if valid.any():
            lo, hi = np.percentile(dd[valid], [2, 98])
            vis = np.clip((dd - lo) / max(hi - lo, 1e-6), 0, 1)
        vis[~valid] = np.nan  # invalido (recortado/sin dato) -> negro
        cmap = plt.cm.viridis.copy(); cmap.set_bad("black")
        axes[i].imshow(vis, cmap=cmap)
        add_grid(axes[i], h, w, "DEPTH — negro=sin profundidad (lo que recorta el ROI)")
        draw_poly(axes[i], poly, h, w)
        i += 1

    out = f"{d}/zed_roi_view.png"
    plt.tight_layout()
    plt.savefig(out, dpi=110)
    print(f"\nGuardado: {out}")


if __name__ == "__main__":
    main()
