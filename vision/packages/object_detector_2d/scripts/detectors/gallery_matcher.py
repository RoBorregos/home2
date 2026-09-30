"""Pure-numpy gallery matching: cosine similarity with a per-class floor and
top1-vs-top2 margin — shared by the benchmark and the production detector as the one implementation. No ROS/ultralytics/torch import, so it stays importable from a plain venv."""

import json
from pathlib import Path

import numpy as np

UNKNOWN = "unknown"

# Calibrated by e2e_calibrate.py against real box-proposer crops (oracle
# crops score better but don't hold up in production) — manifest.json's per-class values override both.
DEFAULT_MIN_SIMILARITY = 0.4
DEFAULT_MARGIN_MIN = 0.04

# A box covering most of the frame is clutter, not an object — shared by
# the runtime detector and gallery_build.py so both crop by the same rule (override via registry.py's max_box_area_frac).
DEFAULT_MAX_BOX_AREA_FRAC = 0.5


def l2_normalize(vectors: np.ndarray) -> np.ndarray:
    vectors = np.asarray(vectors, dtype=np.float32)
    norms = np.linalg.norm(vectors, axis=-1, keepdims=True)
    return vectors / np.clip(norms, 1e-10, None)


class Gallery:
    """name -> L2-normalized embeddings[N_i, D], plus a per-class (min_similarity,
    margin_min) match threshold loaded from manifest.json."""

    def __init__(self, embeddings: dict[str, np.ndarray], manifest: dict | None = None):
        manifest = manifest or {}
        self._label_idx: dict[str, np.ndarray] = {}
        self.thresholds: dict[str, tuple[float, float]] = {}

        labels = []
        rows = []
        for name, vecs in embeddings.items():
            vecs = l2_normalize(vecs)
            start = len(rows)
            rows.extend(vecs)
            labels.extend([name] * len(vecs))
            self._label_idx[name] = np.arange(start, start + len(vecs))
            obj_cfg = (manifest.get("objects") or {}).get(name, {})
            self.thresholds[name] = (
                obj_cfg.get("min_similarity", DEFAULT_MIN_SIMILARITY),
                obj_cfg.get("margin_min", DEFAULT_MARGIN_MIN),
            )

        self.labels = labels
        self.vectors = (
            np.stack(rows).astype(np.float32) if rows else np.zeros((0, 0), np.float32)
        )

    @classmethod
    def load(cls, gallery_dir: str | Path) -> "Gallery":
        gallery_dir = Path(gallery_dir)
        manifest_path = gallery_dir / "manifest.json"
        manifest = (
            json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        )
        embeddings = {}
        for name, obj_cfg in (manifest.get("objects") or {}).items():
            npy_path = gallery_dir / obj_cfg["npy"]
            if npy_path.exists():
                embeddings[name] = np.load(npy_path)
        return cls(embeddings, manifest)

    def match_batch(self, queries: np.ndarray) -> list[tuple[str, float, float]]:
        """queries: [Q, D]. Returns one (label, top1_similarity, margin) per row;
        UNKNOWN when the winner misses its floor/margin. Margin is between CLASSES (best per label), not the two closest vectors, or the runner-up would usually be the same object and collapse it to ~0."""
        n = len(queries)
        if self.vectors.size == 0 or n == 0:
            return [(UNKNOWN, 0.0, 0.0) for _ in range(n)]

        q = l2_normalize(queries)
        sims = q @ self.vectors.T  # [Q, N]

        results = []
        for row in sims:
            per_class = {
                name: float(row[idxs].max()) for name, idxs in self._label_idx.items()
            }
            ranked = sorted(per_class.items(), key=lambda kv: -kv[1])
            top1_label, top1_sim = ranked[0]
            top2_sim = ranked[1][1] if len(ranked) > 1 else -1.0
            margin = top1_sim - top2_sim
            min_sim, margin_min = self.thresholds.get(
                top1_label, (DEFAULT_MIN_SIMILARITY, DEFAULT_MARGIN_MIN)
            )
            label = (
                top1_label
                if (top1_sim >= min_sim and margin >= margin_min)
                else UNKNOWN
            )
            results.append((label, top1_sim, margin))
        return results

    def match(self, query: np.ndarray) -> tuple[str, float, float]:
        return self.match_batch(query[None, :])[0]
