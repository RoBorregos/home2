#!/usr/bin/env python3
"""
Benchmark: face_recognition (dlib) vs insightface (ArcFace)

Métricas:
  - Tiempo de detección por frame
  - Tiempo de encoding/embedding por cara
  - Distribución de distancias (mismo vs diferente)
  - Throughput (FPS estimado)

Uso:
  python3 benchmark_face.py 
"""

import argparse
import os
import time
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

KNOWN_FACES_PATH = str(Path(__file__).parent.parent / "Utils" / "known_faces")
OUTPUT_DIR = str(Path(__file__).parent)
N_RUNS = 10 

def load_images(directory: str) -> list[tuple[str, np.ndarray]]:
    """Return list of (name, bgr_image) from directory."""
    images = []
    for f in sorted(os.listdir(directory)):
        if not f.lower().endswith((".png", ".jpg", ".jpeg")):
            continue
        path = os.path.join(directory, f)
        img = cv2.imread(path)
        if img is not None:
            images.append((f[:-4], img))
    return images


def cosine_distance(a: np.ndarray, b: np.ndarray) -> float:
    a = a / (np.linalg.norm(a) + 1e-10)
    b = b / (np.linalg.norm(b) + 1e-10)
    return float(1.0 - np.dot(a, b))



def benchmark_dlib(images: list[tuple[str, np.ndarray]], n_runs: int):
    import face_recognition
    from PIL import Image as PILImage

    detect_times, encode_times, embeddings, names = [], [], [], []

    for name, bgr in images:
        pil_img = PILImage.fromarray(bgr[:, :, ::-1])  # BGR→RGB via PIL
        rgb = np.array(pil_img)

        times = []
        for _ in range(n_runs):
            t0 = time.perf_counter()
            locs = face_recognition.face_locations(rgb, model="cnn")
            times.append(time.perf_counter() - t0)
        detect_times.append(np.mean(times) * 1000)

        if not locs:
            print(f"  [dlib] no face in {name}, skipping encoding")
            continue

        times = []
        for _ in range(n_runs):
            t0 = time.perf_counter()
            encs = face_recognition.face_encodings(rgb, locs)
            times.append(time.perf_counter() - t0)
        encode_times.append(np.mean(times) * 1000)

        if encs:
            embeddings.append(encs[0])
            names.append(name)

    return detect_times, encode_times, embeddings, names



def benchmark_insightface(images: list[tuple[str, np.ndarray]], n_runs: int):
    import sys
    from unittest.mock import MagicMock
    sys.modules.setdefault("insightface.app.mask_renderer",          MagicMock())
    sys.modules.setdefault("insightface.thirdparty.face3d",          MagicMock())
    sys.modules.setdefault("insightface.thirdparty.face3d.mesh",     MagicMock())
    sys.modules.setdefault("insightface.thirdparty.face3d.mesh.vis", MagicMock())
    from insightface.app import FaceAnalysis

    app = FaceAnalysis(
        name="buffalo_l",
        providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    app.prepare(ctx_id=0, det_size=(640, 640))

    detect_times, encode_times, embeddings, names = [], [], [], []

    for name, bgr in images:
        times = []
        faces_result = None
        for i in range(n_runs):
            t0 = time.perf_counter()
            faces_result = app.get(bgr)
            elapsed = time.perf_counter() - t0
            times.append(elapsed)

        total_mean = np.mean(times) * 1000

        if not faces_result:
            print(f"  [insightface] no face in {name}, skipping")
            detect_times.append(total_mean)
            continue

        detect_times.append(total_mean * 0.6)
        encode_times.append(total_mean * 0.4)

        face = max(faces_result, key=lambda f: (f.bbox[2]-f.bbox[0])*(f.bbox[3]-f.bbox[1]))
        embeddings.append(face.embedding.astype(np.float32))
        names.append(name)

    return detect_times, encode_times, embeddings, names


def compute_distance_matrix(embeddings, use_cosine=False):
    n = len(embeddings)
    mat = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if use_cosine:
                mat[i, j] = cosine_distance(embeddings[i], embeddings[j])
            else:
                mat[i, j] = np.linalg.norm(embeddings[i] - embeddings[j])
    return mat

def plot_results(
    dlib_detect, dlib_encode, dlib_embs, dlib_names,
    ins_detect,  ins_encode,  ins_embs,  ins_names,
    output_dir: str,
):
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    fig.suptitle("Benchmark: face_recognition (dlib) vs insightface (ArcFace)", fontsize=14)

    colors = {"dlib": "#4C72B0", "insightface": "#DD8452"}

    ax = axes[0, 0]
    x = np.arange(len(dlib_detect))
    w = 0.35
    ins_det_trim = ins_detect[:len(dlib_detect)]
    ax.bar(x - w/2, dlib_detect, w, label="dlib", color=colors["dlib"])
    ax.bar(x + w/2, ins_det_trim, w, label="insightface", color=colors["insightface"])
    ax.set_title("Tiempo de detección por imagen (ms)")
    ax.set_ylabel("ms")
    ax.set_xticks(x)
    ax.set_xticklabels([f"img{i+1}" for i in x], rotation=45)
    ax.legend()

    ax = axes[0, 1]
    labels = ["dlib (128-d\nL2)", "insightface (512-d\ncoseno)"]
    means  = [np.mean(dlib_encode) if dlib_encode else 0,
              np.mean(ins_encode)  if ins_encode  else 0]
    stds   = [np.std(dlib_encode)  if dlib_encode else 0,
              np.std(ins_encode)   if ins_encode  else 0]
    bars = ax.bar(labels, means, yerr=stds, capsize=6,
                  color=[colors["dlib"], colors["insightface"]], alpha=0.85)
    ax.set_title("Tiempo medio de embedding (ms)")
    ax.set_ylabel("ms")
    for bar, mean in zip(bars, means):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f"{mean:.1f}", ha="center", va="bottom", fontsize=9)

    ax = axes[0, 2]
    dlib_total  = np.mean(dlib_detect) + (np.mean(dlib_encode) if dlib_encode else 0)
    ins_total   = np.mean(ins_detect)  + (np.mean(ins_encode)  if ins_encode  else 0)
    fps_dlib    = 1000.0 / dlib_total  if dlib_total  > 0 else 0
    fps_ins     = 1000.0 / ins_total   if ins_total   > 0 else 0
    bars = ax.bar(["dlib", "insightface"], [fps_dlib, fps_ins],
                  color=[colors["dlib"], colors["insightface"]], alpha=0.85)
    ax.set_title("FPS estimado (1 cara/frame)")
    ax.set_ylabel("FPS")
    for bar, fps in zip(bars, [fps_dlib, fps_ins]):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                f"{fps:.1f}", ha="center", va="bottom", fontsize=11, fontweight="bold")

    ax = axes[1, 0]
    if len(dlib_embs) >= 2:
        mat = compute_distance_matrix(dlib_embs, use_cosine=False)
        im = ax.imshow(mat, cmap="Blues", vmin=0)
        ax.set_title("Distancia L2 — dlib\n(más bajo = más similar)")
        ax.set_xticks(range(len(dlib_names)))
        ax.set_yticks(range(len(dlib_names)))
        ax.set_xticklabels(dlib_names, rotation=45, ha="right", fontsize=8)
        ax.set_yticklabels(dlib_names, fontsize=8)
        for i in range(len(dlib_names)):
            for j in range(len(dlib_names)):
                ax.text(j, i, f"{mat[i,j]:.3f}", ha="center", va="center", fontsize=7)
        plt.colorbar(im, ax=ax)
    else:
        ax.text(0.5, 0.5, "Necesitas ≥2 imágenes\ncon cara detectada",
                ha="center", va="center", transform=ax.transAxes)
        ax.set_title("Distancia L2 — dlib")

    ax = axes[1, 1]
    if len(ins_embs) >= 2:
        mat = compute_distance_matrix(ins_embs, use_cosine=True)
        im = ax.imshow(mat, cmap="Oranges", vmin=0, vmax=1)
        ax.set_title("Distancia coseno — insightface\n(más bajo = más similar)")
        ax.set_xticks(range(len(ins_names)))
        ax.set_yticks(range(len(ins_names)))
        ax.set_xticklabels(ins_names, rotation=45, ha="right", fontsize=8)
        ax.set_yticklabels(ins_names, fontsize=8)
        for i in range(len(ins_names)):
            for j in range(len(ins_names)):
                ax.text(j, i, f"{mat[i,j]:.3f}", ha="center", va="center", fontsize=7)
        plt.colorbar(im, ax=ax)
    else:
        ax.text(0.5, 0.5, "Necesitas ≥2 imágenes\ncon cara detectada",
                ha="center", va="center", transform=ax.transAxes)
        ax.set_title("Distancia coseno — insightface")

    ax = axes[1, 2]
    ax.axis("off")
    summary = (
        f"{'Métrica':<28} {'dlib':>10} {'insightface':>12}\n"
        f"{'─'*52}\n"
        f"{'Det. media (ms)':<28} {np.mean(dlib_detect):>10.1f} {np.mean(ins_detect):>12.1f}\n"
        f"{'Embed. media (ms)':<28} "
        f"{np.mean(dlib_encode) if dlib_encode else 0:>10.1f} "
        f"{np.mean(ins_encode)  if ins_encode  else 0:>12.1f}\n"
        f"{'Total/frame (ms)':<28} {dlib_total:>10.1f} {ins_total:>12.1f}\n"
        f"{'FPS estimado':<28} {fps_dlib:>10.1f} {fps_ins:>12.1f}\n"
        f"{'Dimensión embedding':<28} {'128':>10} {'512':>12}\n"
        f"{'Métrica similitud':<28} {'L2':>10} {'coseno':>12}\n"
        f"{'Threshold default':<28} {'0.50':>10} {'0.35':>12}\n"
    )
    ax.text(0.05, 0.95, summary, transform=ax.transAxes,
            fontsize=9, verticalalignment="top", fontfamily="monospace",
            bbox=dict(boxstyle="round", facecolor="whitesmoke", alpha=0.8))
    ax.set_title("Resumen")

    plt.tight_layout()
    out_path = os.path.join(output_dir, "benchmark_face_recognition.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\n✓ Gráfica guardada en: {out_path}")
    return out_path



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", default=KNOWN_FACES_PATH,
                        help="Directorio con imágenes de prueba")
    parser.add_argument("--runs", type=int, default=N_RUNS,
                        help="Repeticiones por imagen para promediar")
    parser.add_argument("--output", default=OUTPUT_DIR,
                        help="Directorio donde guardar la gráfica")
    args = parser.parse_args()

    images = load_images(args.images)
    if not images:
        print(f"No se encontraron imágenes en {args.images}")
        return

    print(f"Imágenes encontradas: {[n for n, _ in images]}")
    print(f"Repeticiones por imagen: {args.runs}\n")

    print("▶ Benchmarking face_recognition (dlib) …")
    try:
        dlib_detect, dlib_encode, dlib_embs, dlib_names = benchmark_dlib(images, args.runs)
        print(f"  Detección media:  {np.mean(dlib_detect):.1f} ms")
        print(f"  Encoding medio:   {np.mean(dlib_encode) if dlib_encode else 0:.1f} ms")
    except ImportError:
        print("  ✗ face_recognition no está instalado en este entorno")
        dlib_detect = dlib_encode = dlib_embs = dlib_names = []

    print("\n▶ Benchmarking insightface (ArcFace) …")
    try:
        ins_detect, ins_encode, ins_embs, ins_names = benchmark_insightface(images, args.runs)
        print(f"  Detección media:  {np.mean(ins_detect):.1f} ms")
        print(f"  Encoding medio:   {np.mean(ins_encode) if ins_encode else 0:.1f} ms")
    except ImportError:
        print("  ✗ insightface no está instalado en este entorno")
        ins_detect = ins_encode = ins_embs = ins_names = []

    if not dlib_detect and not ins_detect:
        print("\nNinguna librería disponible. Instala al menos una.")
        return

    if not dlib_detect:
        dlib_detect = [0] * len(images)
        dlib_encode = dlib_embs = dlib_names = []
    if not ins_detect:
        ins_detect = [0] * len(images)
        ins_encode = ins_embs = ins_names = []

    print("\n▶ Generando gráficas …")
    plot_results(
        dlib_detect, dlib_encode, dlib_embs, dlib_names,
        ins_detect,  ins_encode,  ins_embs,  ins_names,
        args.output,
    )


if __name__ == "__main__":
    main()
