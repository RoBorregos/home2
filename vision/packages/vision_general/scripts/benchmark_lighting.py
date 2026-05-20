#!/usr/bin/env python3
"""
Benchmark de condiciones de iluminación:
  face_recognition (dlib) vs insightface (ArcFace)

Condiciones simuladas:
  - Normal
  - Poca luz (low light)
  - Contraluz (backlight)
  - Sobreexposición (overexposed)
  - Luz cálida (warm)
  - Luz fría / fluorescente (cool)
  - Niebla / neblina (haze)
  - Alto contraste (high contrast)

Métricas por condición:
  - ¿Se detectó la cara? (detection rate)
  - Confianza de reconocimiento (distancia / similitud)
  - Tiempo de inferencia (ms)

Uso:
  python3 benchmark_lighting.py --images /path/to/images --known /path/to/known_faces
  python3 benchmark_lighting.py   # usa Utils/known_faces por defecto
"""

import argparse
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

KNOWN_FACES_PATH = str(Path(__file__).parent.parent / "Utils" / "known_faces")
OUTPUT_DIR       = str(Path(__file__).parent)
N_RUNS           = 5   # repeticiones para promediar tiempos


# ─────────────────────────────────────────────────────────────────────────────
# Filtros de iluminación
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class LightingFilter:
    name:  str
    label: str          # nombre corto para ejes
    color: str          # color de la barra
    apply: Callable     # (bgr: np.ndarray) -> np.ndarray


def _clamp(img: np.ndarray) -> np.ndarray:
    return np.clip(img, 0, 255).astype(np.uint8)


def _apply_backlight(img: np.ndarray) -> np.ndarray:
    """Simula contraluz: cara oscurecida, halo brillante en el borde."""
    out = img.astype(np.float32)
    h, w = out.shape[:2]
    # Gradiente radial: centro oscuro, bordes muy brillantes
    Y, X = np.ogrid[:h, :w]
    cx, cy = w / 2, h / 2
    dist = np.sqrt((X - cx)**2 + (Y - cy)**2)
    max_dist = np.sqrt(cx**2 + cy**2)
    mask = (dist / max_dist)             # 0 en centro, 1 en borde
    mask = mask[:, :, np.newaxis]
    # oscurecer cara (centro) y sobreexponer bordes
    out = out * 0.15 + out * mask * 2.5
    return _clamp(out)


def _apply_warm(img: np.ndarray) -> np.ndarray:
    """Tinte cálido: boost rojo/verde, reducir azul."""
    out = img.astype(np.float32)
    out[:, :, 0] = np.clip(out[:, :, 0] * 0.75, 0, 255)   # B
    out[:, :, 1] = np.clip(out[:, :, 1] * 1.05, 0, 255)   # G
    out[:, :, 2] = np.clip(out[:, :, 2] * 1.3,  0, 255)   # R
    return _clamp(out)


def _apply_cool(img: np.ndarray) -> np.ndarray:
    """Tinte frío / fluorescente: boost azul, reducir rojo."""
    out = img.astype(np.float32)
    out[:, :, 0] = np.clip(out[:, :, 0] * 1.4,  0, 255)   # B
    out[:, :, 1] = np.clip(out[:, :, 1] * 1.05, 0, 255)   # G
    out[:, :, 2] = np.clip(out[:, :, 2] * 0.75, 0, 255)   # R
    return _clamp(out)


def _apply_haze(img: np.ndarray) -> np.ndarray:
    """Neblina: mezcla con blanco."""
    out = img.astype(np.float32)
    white = np.full_like(out, 220.0)
    return _clamp(out * 0.5 + white * 0.5)


def _apply_high_contrast(img: np.ndarray) -> np.ndarray:
    """Alto contraste con CLAHE agresivo."""
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=6.0, tileGridSize=(4, 4))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


FILTERS: list[LightingFilter] = [
    LightingFilter(
        name="Normal", label="Normal", color="#4E8098",
        apply=lambda img: img.copy(),
    ),
    LightingFilter(
        name="Poca luz", label="Poca\nluz", color="#2D2D2D",
        apply=lambda img: _clamp(img.astype(np.float32) * 0.25),
    ),
    LightingFilter(
        name="Muy poca luz", label="Muy\npoca luz", color="#1A1A1A",
        apply=lambda img: _clamp(img.astype(np.float32) * 0.10),
    ),
    LightingFilter(
        name="Contraluz", label="Contra-\nluz", color="#8B4513",
        apply=_apply_backlight,
    ),
    LightingFilter(
        name="Sobreexposición", label="Sobre-\nexpos.", color="#FFD700",
        apply=lambda img: _clamp(img.astype(np.float32) * 2.2),
    ),
    LightingFilter(
        name="Luz cálida", label="Cálida", color="#E07B39",
        apply=_apply_warm,
    ),
    LightingFilter(
        name="Luz fría", label="Fría", color="#6EB5C0",
        apply=_apply_cool,
    ),
    LightingFilter(
        name="Niebla", label="Niebla", color="#B0C4C8",
        apply=_apply_haze,
    ),
    LightingFilter(
        name="Alto contraste", label="Alto\ncontraste", color="#555555",
        apply=_apply_high_contrast,
    ),
]


# ─────────────────────────────────────────────────────────────────────────────
# Resultado por condición
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ConditionResult:
    filter_name:  str
    detected:     bool  = False
    confidence:   float = 0.0   # similitud [0,1] — 1 = idéntico
    infer_ms:     float = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Backend dlib
# ─────────────────────────────────────────────────────────────────────────────

def run_dlib(
    known_embeddings: list[np.ndarray],
    images: list[tuple[str, np.ndarray]],
    n_runs: int,
) -> dict[str, list[ConditionResult]]:
    import face_recognition

    results: dict[str, list[ConditionResult]] = {}

    for img_name, bgr in images:
        img_results = []
        for filt in FILTERS:
            filtered_bgr = filt.apply(bgr)
            rgb = filtered_bgr[:, :, ::-1].copy()

            times = []
            locs  = []
            for _ in range(n_runs):
                t0   = time.perf_counter()
                locs = face_recognition.face_locations(rgb, model="cnn")
                times.append(time.perf_counter() - t0)
            infer_ms = np.mean(times) * 1000

            res = ConditionResult(filter_name=filt.name, infer_ms=infer_ms)

            if locs:
                encs = face_recognition.face_encodings(rgb, locs)
                if encs and known_embeddings:
                    distances  = face_recognition.face_distance(known_embeddings, encs[0])
                    best_dist  = float(np.min(distances))
                    # convertir distancia L2 a similitud [0,1]
                    confidence = max(0.0, 1.0 - best_dist)
                    res.detected   = True
                    res.confidence = confidence

            img_results.append(res)
            status = f"det={res.detected} conf={res.confidence:.3f} t={infer_ms:.0f}ms"
            print(f"    [{filt.name:18s}] {status}")

        results[img_name] = img_results

    return results


# ─────────────────────────────────────────────────────────────────────────────
# Backend insightface
# ─────────────────────────────────────────────────────────────────────────────

def _cosine_sim(a: np.ndarray, b: np.ndarray) -> float:
    a = a / (np.linalg.norm(a) + 1e-10)
    b = b / (np.linalg.norm(b) + 1e-10)
    return float(np.dot(a, b))


def run_insightface(
    known_embeddings: list[np.ndarray],
    images: list[tuple[str, np.ndarray]],
    n_runs: int,
) -> dict[str, list[ConditionResult]]:
    from insightface.app import FaceAnalysis

    app = FaceAnalysis(
        name="buffalo_l",
        providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )
    app.prepare(ctx_id=0, det_size=(640, 640))

    results: dict[str, list[ConditionResult]] = {}

    for img_name, bgr in images:
        img_results = []
        for filt in FILTERS:
            filtered_bgr = filt.apply(bgr)

            times       = []
            faces_result = []
            for _ in range(n_runs):
                t0           = time.perf_counter()
                faces_result = app.get(filtered_bgr)
                times.append(time.perf_counter() - t0)
            infer_ms = np.mean(times) * 1000

            res = ConditionResult(filter_name=filt.name, infer_ms=infer_ms)

            if faces_result:
                face  = max(faces_result,
                            key=lambda f: (f.bbox[2]-f.bbox[0])*(f.bbox[3]-f.bbox[1]))
                query = face.embedding.astype(np.float32)

                if known_embeddings:
                    sims      = [_cosine_sim(query, k) for k in known_embeddings]
                    best_sim  = float(max(sims))
                    confidence = max(0.0, best_sim)   # ya está en [0,1]
                    res.detected   = True
                    res.confidence = confidence

            img_results.append(res)
            status = f"det={res.detected} conf={res.confidence:.3f} t={infer_ms:.0f}ms"
            print(f"    [{filt.name:18s}] {status}")

        results[img_name] = img_results

    return results


# ─────────────────────────────────────────────────────────────────────────────
# Cargar imágenes y embeddings de referencia
# ─────────────────────────────────────────────────────────────────────────────

def load_images(directory: str) -> list[tuple[str, np.ndarray]]:
    images = []
    for f in sorted(os.listdir(directory)):
        if not f.lower().endswith((".png", ".jpg", ".jpeg")):
            continue
        img = cv2.imread(os.path.join(directory, f))
        if img is not None:
            images.append((f[:-4], img))
    return images


def load_known_dlib(known_path: str) -> list[np.ndarray]:
    import face_recognition
    embeddings = []
    for f in sorted(os.listdir(known_path)):
        if not f.lower().endswith((".png", ".jpg", ".jpeg")):
            continue
        img = face_recognition.load_image_file(os.path.join(known_path, f))
        encs = face_recognition.face_encodings(img)
        if encs:
            embeddings.append(encs[0])
    return embeddings


def load_known_insightface(app, known_path: str) -> list[np.ndarray]:
    embeddings = []
    for f in sorted(os.listdir(known_path)):
        if not f.lower().endswith((".png", ".jpg", ".jpeg")):
            continue
        img   = cv2.imread(os.path.join(known_path, f))
        faces = app.get(img)
        if faces:
            face = max(faces, key=lambda x: (x.bbox[2]-x.bbox[0])*(x.bbox[3]-x.bbox[1]))
            embeddings.append(face.embedding.astype(np.float32))
    return embeddings


# ─────────────────────────────────────────────────────────────────────────────
# Guardar imágenes de preview de cada filtro
# ─────────────────────────────────────────────────────────────────────────────

def save_filter_previews(images: list[tuple[str, np.ndarray]], output_dir: str) -> None:
    if not images:
        return
    name, bgr = images[0]
    fig, axes = plt.subplots(2, 5, figsize=(20, 8))
    fig.suptitle(f"Filtros de iluminación aplicados a '{name}'", fontsize=13)
    axes = axes.flatten()
    for i, filt in enumerate(FILTERS):
        filtered = filt.apply(bgr)
        axes[i].imshow(cv2.cvtColor(filtered, cv2.COLOR_BGR2RGB))
        axes[i].set_title(filt.name, fontsize=10)
        axes[i].axis("off")
    for j in range(len(FILTERS), len(axes)):
        axes[j].axis("off")
    plt.tight_layout()
    out = os.path.join(output_dir, "lighting_previews.png")
    plt.savefig(out, dpi=120, bbox_inches="tight")
    print(f"✓ Preview de filtros guardado: {out}")
    plt.close()


# ─────────────────────────────────────────────────────────────────────────────
# Plots principales
# ─────────────────────────────────────────────────────────────────────────────

def plot_lighting_results(
    dlib_results: dict[str, list[ConditionResult]] | None,
    ins_results:  dict[str, list[ConditionResult]] | None,
    output_dir:   str,
) -> None:
    filter_labels = [f.label for f in FILTERS]
    filter_colors = [f.color for f in FILTERS]
    n_filters     = len(FILTERS)
    x             = np.arange(n_filters)
    width         = 0.35

    # ── Figura 1: Confianza de reconocimiento por condición ───────────────────
    fig1, axes1 = plt.subplots(1, 2, figsize=(18, 6), sharey=True)
    fig1.suptitle(
        "Confianza de reconocimiento por condición de luz\n"
        "(promedio sobre todas las imágenes de prueba)",
        fontsize=13,
    )

    for ax, res_dict, title, bar_color in [
        (axes1[0], dlib_results,  "face_recognition — dlib (128-d L2→sim)", "#4C72B0"),
        (axes1[1], ins_results,   "insightface — ArcFace (512-d coseno)",    "#DD8452"),
    ]:
        if res_dict is None:
            ax.text(0.5, 0.5, "No disponible", ha="center", va="center",
                    transform=ax.transAxes, fontsize=14)
            ax.set_title(title)
            continue

        # promedio de confianza por filtro entre todas las imágenes
        conf_per_filter = []
        det_rate_filter = []
        for fi in range(n_filters):
            confs = [img_res[fi].confidence for img_res in res_dict.values()]
            dets  = [img_res[fi].detected   for img_res in res_dict.values()]
            conf_per_filter.append(np.mean(confs))
            det_rate_filter.append(np.mean([1 if d else 0 for d in dets]))

        bars = ax.bar(x, conf_per_filter, color=filter_colors, alpha=0.85, edgecolor="white")
        ax.set_xticks(x)
        ax.set_xticklabels(filter_labels, fontsize=9)
        ax.set_ylim(0, 1.15)
        ax.set_ylabel("Confianza (0–1)")
        ax.set_title(title, fontsize=10)
        ax.axhline(0.5, color="red", linestyle="--", linewidth=1, label="Threshold 0.5")
        ax.axhline(0.35, color="orange", linestyle=":", linewidth=1, label="Threshold 0.35")
        ax.legend(fontsize=8)

        # etiquetas: confianza + detection rate
        for bar, conf, det in zip(bars, conf_per_filter, det_rate_filter):
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.02,
                f"{conf:.2f}\n({det*100:.0f}%det)",
                ha="center", va="bottom", fontsize=7.5, fontweight="bold",
            )

    fig1.tight_layout()
    out1 = os.path.join(output_dir, "lighting_confidence.png")
    fig1.savefig(out1, dpi=150, bbox_inches="tight")
    print(f"✓ Gráfica de confianza: {out1}")
    plt.close(fig1)

    # ── Figura 2: Comparación dlib vs insightface lado a lado ─────────────────
    if dlib_results is not None and ins_results is not None:
        # usar la primera imagen como representativa
        img_name   = next(iter(dlib_results))
        dlib_confs = [r.confidence for r in dlib_results[img_name]]
        ins_confs  = [r.confidence for r in ins_results[img_name]]
        dlib_det   = [r.detected   for r in dlib_results[img_name]]
        ins_det    = [r.detected   for r in ins_results[img_name]]
        dlib_ms    = [r.infer_ms   for r in dlib_results[img_name]]
        ins_ms     = [r.infer_ms   for r in ins_results[img_name]]

        fig2, axes2 = plt.subplots(1, 3, figsize=(20, 6))
        fig2.suptitle(
            f"dlib vs insightface — imagen: '{img_name}'",
            fontsize=13,
        )

        # subplot 1: confianza
        ax = axes2[0]
        ax.bar(x - width/2, dlib_confs, width, label="dlib",        color="#4C72B0", alpha=0.85)
        ax.bar(x + width/2, ins_confs,  width, label="insightface",  color="#DD8452", alpha=0.85)
        ax.set_xticks(x)
        ax.set_xticklabels(filter_labels, fontsize=8)
        ax.set_ylim(0, 1.2)
        ax.set_ylabel("Confianza")
        ax.set_title("Confianza de reconocimiento")
        ax.axhline(0.5,  color="#4C72B0", linestyle="--", linewidth=1, alpha=0.6, label="dlib threshold")
        ax.axhline(0.35, color="#DD8452", linestyle=":",  linewidth=1, alpha=0.6, label="insight threshold")
        ax.legend(fontsize=8)

        # subplot 2: detección (barras apiladas)
        ax = axes2[1]
        dlib_det_int = [1 if d else 0 for d in dlib_det]
        ins_det_int  = [1 if d else 0 for d in ins_det]
        ax.bar(x - width/2, dlib_det_int, width, label="dlib",       color="#4C72B0", alpha=0.85)
        ax.bar(x + width/2, ins_det_int,  width, label="insightface", color="#DD8452", alpha=0.85)
        ax.set_xticks(x)
        ax.set_xticklabels(filter_labels, fontsize=8)
        ax.set_ylim(0, 1.4)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(["No detectado", "Detectado"])
        ax.set_title("Detección (0/1) por condición")
        ax.legend(fontsize=8)

        # subplot 3: tiempo de inferencia
        ax = axes2[2]
        ax.bar(x - width/2, dlib_ms, width, label="dlib",        color="#4C72B0", alpha=0.85)
        ax.bar(x + width/2, ins_ms,  width, label="insightface",  color="#DD8452", alpha=0.85)
        ax.set_xticks(x)
        ax.set_xticklabels(filter_labels, fontsize=8)
        ax.set_ylabel("ms")
        ax.set_title("Tiempo de inferencia (ms)")
        ax.legend(fontsize=8)

        fig2.tight_layout()
        out2 = os.path.join(output_dir, "lighting_comparison.png")
        fig2.savefig(out2, dpi=150, bbox_inches="tight")
        print(f"✓ Comparación dlib vs insightface: {out2}")
        plt.close(fig2)

    # ── Figura 3: Heatmap confianza — todas las imágenes x todos los filtros ──
    for res_dict, title, cmap, fname in [
        (dlib_results, "dlib — confianza por imagen × condición",        "Blues",   "heatmap_dlib.png"),
        (ins_results,  "insightface — confianza por imagen × condición", "Oranges", "heatmap_insightface.png"),
    ]:
        if res_dict is None or len(res_dict) < 2:
            continue

        img_names = list(res_dict.keys())
        mat = np.array([
            [res_dict[n][fi].confidence for fi in range(n_filters)]
            for n in img_names
        ])

        fig3, ax = plt.subplots(figsize=(14, max(3, len(img_names) * 0.8 + 2)))
        im = ax.imshow(mat, cmap=cmap, vmin=0, vmax=1, aspect="auto")
        ax.set_xticks(range(n_filters))
        ax.set_xticklabels([f.name for f in FILTERS], rotation=40, ha="right", fontsize=9)
        ax.set_yticks(range(len(img_names)))
        ax.set_yticklabels(img_names, fontsize=9)
        ax.set_title(title, fontsize=12)
        plt.colorbar(im, ax=ax, label="Confianza")
        for i in range(len(img_names)):
            for j in range(n_filters):
                val  = mat[i, j]
                det  = res_dict[img_names[i]][j].detected
                text = f"{val:.2f}" if det else "✗"
                ax.text(j, i, text, ha="center", va="center",
                        fontsize=7.5, color="black" if val < 0.6 else "white")
        fig3.tight_layout()
        out3 = os.path.join(output_dir, fname)
        fig3.savefig(out3, dpi=150, bbox_inches="tight")
        print(f"✓ Heatmap: {out3}")
        plt.close(fig3)

    # ── Figura 4: Radar chart — robustez global por condición ─────────────────
    _plot_radar(dlib_results, ins_results, output_dir)


def _plot_radar(
    dlib_results: dict | None,
    ins_results:  dict | None,
    output_dir:   str,
) -> None:
    categories  = [f.name for f in FILTERS]
    n_cat       = len(categories)
    angles      = np.linspace(0, 2 * np.pi, n_cat, endpoint=False).tolist()
    angles     += angles[:1]  # cerrar el polígono

    fig, ax = plt.subplots(figsize=(9, 9), subplot_kw=dict(polar=True))
    ax.set_title("Robustez global por condición de luz\n(confianza media)", fontsize=13, pad=20)

    for res_dict, label, color in [
        (dlib_results, "dlib",        "#4C72B0"),
        (ins_results,  "insightface", "#DD8452"),
    ]:
        if res_dict is None:
            continue
        values = []
        for fi in range(n_cat):
            confs  = [r[fi].confidence for r in res_dict.values()]
            values.append(float(np.mean(confs)))
        values += values[:1]
        ax.plot(angles, values, color=color, linewidth=2, label=label)
        ax.fill(angles, values, color=color, alpha=0.15)

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(categories, fontsize=9)
    ax.set_ylim(0, 1)
    ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_yticklabels(["0.2", "0.4", "0.6", "0.8", "1.0"], fontsize=7)
    ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1), fontsize=11)

    out = os.path.join(output_dir, "lighting_radar.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"✓ Radar chart: {out}")
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", default=KNOWN_FACES_PATH,
                        help="Directorio con imágenes de prueba")
    parser.add_argument("--known", default=KNOWN_FACES_PATH,
                        help="Directorio con caras de referencia (enrolled)")
    parser.add_argument("--runs", type=int, default=N_RUNS)
    parser.add_argument("--output", default=OUTPUT_DIR)
    parser.add_argument("--skip-dlib",        action="store_true")
    parser.add_argument("--skip-insightface", action="store_true")
    args = parser.parse_args()

    images = load_images(args.images)
    if not images:
        print(f"No se encontraron imágenes en {args.images}")
        return
    print(f"Imágenes de prueba: {[n for n, _ in images]}")

    # ── Preview de filtros ────────────────────────────────────────────────────
    save_filter_previews(images, args.output)

    dlib_results = None
    ins_results  = None

    # ── dlib ──────────────────────────────────────────────────────────────────
    if not args.skip_dlib:
        print("\n▶ Cargando embeddings de referencia (dlib) …")
        try:
            known_dlib = load_known_dlib(args.known)
            print(f"  {len(known_dlib)} caras cargadas")
            print("▶ Benchmarking dlib por condición de luz …")
            dlib_results = run_dlib(known_dlib, images, args.runs)
        except ImportError:
            print("  ✗ face_recognition no disponible en este entorno")

    # ── insightface ───────────────────────────────────────────────────────────
    if not args.skip_insightface:
        print("\n▶ Cargando embeddings de referencia (insightface) …")
        try:
            from insightface.app import FaceAnalysis
            app = FaceAnalysis(
                name="buffalo_l",
                providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
            )
            app.prepare(ctx_id=0, det_size=(640, 640))
            known_ins = load_known_insightface(app, args.known)
            print(f"  {len(known_ins)} caras cargadas")
            print("▶ Benchmarking insightface por condición de luz …")
            ins_results = run_insightface(known_ins, images, args.runs)
        except ImportError:
            print("  ✗ insightface no disponible en este entorno")

    if dlib_results is None and ins_results is None:
        print("\nNinguna librería disponible. Instala face_recognition o insightface.")
        return

    # ── Gráficas ──────────────────────────────────────────────────────────────
    print("\n▶ Generando gráficas …")
    plot_lighting_results(dlib_results, ins_results, args.output)

    print("\n── Resumen por condición ──────────────────────────────────────────")
    header = f"{'Condición':<20}"
    if dlib_results:
        header += f"  {'dlib conf':>10} {'dlib det%':>9}"
    if ins_results:
        header += f"  {'insight conf':>12} {'insight det%':>12}"
    print(header)
    print("─" * len(header))

    sample_img = next(iter(dlib_results or ins_results))
    for fi, filt in enumerate(FILTERS):
        row = f"{filt.name:<20}"
        if dlib_results:
            r = dlib_results[sample_img][fi]
            row += f"  {r.confidence:>10.3f} {'✓' if r.detected else '✗':>9}"
        if ins_results:
            r = ins_results[sample_img][fi]
            row += f"  {r.confidence:>12.3f} {'✓' if r.detected else '✗':>12}"
        print(row)


if __name__ == "__main__":
    main()
