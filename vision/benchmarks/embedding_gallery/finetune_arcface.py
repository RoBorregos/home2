#!/usr/bin/env python3
"""ArcFace fine-tune — the issue explicitly named "ArcFace or triplet loss";
finetune_head.py only tried triplet (+3pt recall on ORACLE crops, not
adopted). This trains a small linear projection with additive angular
margin loss (Deng et al., ArcFace) on REAL box-proposer crops from
RCW2026_v2's TRAIN split — not oracle ground-truth crops, and not the TEST
split used for threshold calibration/evaluation, so this stays honest.

Design: the ArcFace classifier weight matrix is TRAINING-ONLY scaffolding,
discarded after training. Inference still matches the trained projection's
output against a per-object gallery via cosine similarity
(gallery_matcher.Gallery) — same as the frozen backbone — so the "add a
genuinely new object without retraining" property is preserved; only
existing objects' embeddings get reshaped by the projection.

Evaluated by reprojecting e2e_calibrate.py's already-cached TEST-split real
crops (results/e2e_crops_cache.npz) through the trained head — no need to
re-run the box proposer for evaluation, only for the one-time TRAIN-split
collection (~15-18min on the Orin, cached separately so reruns are fast).

Usage:
    python3 finetune_arcface.py --source ~/Downloads/RCW2026_v2 --n-images 400
    python3 finetune_arcface.py --from-cache  # reuse cached train crops
"""

import argparse
import copy
import json
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[2]
        / "packages"
        / "object_detector_2d"
        / "scripts"
        / "detectors"
    ),
)

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from e2e_calibrate import (
    CACHE_PATH as TEST_CACHE_PATH,
    collect_real_crops,
    load_cached_crops,
    optimize_global,
    optimize_per_class,
)

TRAIN_CACHE_PATH = Path(__file__).parent / "results" / "e2e_crops_cache_train.npz"


class ArcFaceHead(nn.Module):
    """Frozen-embedding -> compact projection, trained via an ArcFace
    (additive angular margin) classification loss. `classifier_weight` is
    only used to compute the training loss — never touched at inference."""

    def __init__(
        self,
        in_dim: int,
        num_classes: int,
        out_dim: int = 128,
        dropout: float = 0.3,
        scale: float = 16.0,
        margin: float = 0.3,
    ):
        super().__init__()
        self.dropout = nn.Dropout(dropout)
        self.proj = nn.Linear(in_dim, out_dim, bias=False)
        nn.init.orthogonal_(self.proj.weight)
        self.classifier_weight = nn.Parameter(torch.randn(num_classes, out_dim) * 0.01)
        self.scale = scale
        self.margin = margin

    def project(self, x: torch.Tensor) -> torch.Tensor:
        return F.normalize(self.proj(self.dropout(x)), dim=-1)

    def forward(self, x: torch.Tensor, labels: torch.Tensor) -> torch.Tensor:
        emb = self.project(x)
        W = F.normalize(self.classifier_weight, dim=-1)
        cos_theta = (emb @ W.T).clamp(-1 + 1e-7, 1 - 1e-7)
        theta = torch.acos(cos_theta)
        target_logit = torch.cos(theta + self.margin)
        one_hot = F.one_hot(labels, num_classes=W.shape[0]).float()
        logits = cos_theta * (1 - one_hot) + target_logit * one_hot
        return logits * self.scale


def split_train_val(labels: list[str], emb: np.ndarray, val_frac: float, seed: int):
    rng = np.random.default_rng(seed)
    by_class: dict[str, list[int]] = {}
    for i, label in enumerate(labels):
        by_class.setdefault(label, []).append(i)

    train_idx, val_idx = [], []
    for label, idxs in by_class.items():
        idxs = np.array(idxs)
        rng.shuffle(idxs)
        n_val = max(1, round(len(idxs) * val_frac)) if len(idxs) >= 2 else 0
        val_idx.extend(idxs[:n_val].tolist())
        train_idx.extend(idxs[n_val:].tolist())
    return np.array(train_idx), np.array(val_idx)


def centroid_val_accuracy(
    head: ArcFaceHead, train_labels, train_emb, val_labels, val_emb
) -> float:
    head.eval()
    with torch.no_grad():
        classes = sorted(set(train_labels))
        train_proj = head.project(train_emb)
        centroids = []
        for c in classes:
            idx = [i for i, label in enumerate(train_labels) if label == c]
            centroids.append(
                F.normalize(train_proj[idx].mean(dim=0, keepdim=True), dim=-1)
            )
        C = torch.cat(centroids, dim=0)
        val_proj = head.project(val_emb)
        sims = val_proj @ C.T
        preds = sims.argmax(dim=1).tolist()
        correct = sum(1 for p, label in zip(preds, val_labels) if classes[p] == label)
    head.train()
    return correct / len(val_labels) if val_labels else 0.0


def train_arcface(
    train_labels: list[str],
    train_emb: np.ndarray,
    epochs: int,
    lr: float,
    out_dim: int,
    dropout: float,
    weight_decay: float,
    val_frac: float,
    patience: int,
    scale: float,
    margin: float,
):
    classes = sorted(set(train_labels))
    label_to_id = {c: i for i, c in enumerate(classes)}

    idx_train, idx_val = split_train_val(train_labels, train_emb, val_frac, seed=0)
    X_train = F.normalize(
        torch.tensor(train_emb[idx_train], dtype=torch.float32), dim=-1
    )
    y_train_str = [train_labels[i] for i in idx_train]
    y_train = torch.tensor(
        [label_to_id[label] for label in y_train_str], dtype=torch.long
    )
    X_val = F.normalize(torch.tensor(train_emb[idx_val], dtype=torch.float32), dim=-1)
    y_val_str = [train_labels[i] for i in idx_val]

    print(
        f"[arcface] {len(X_train)} train / {len(X_val)} val crops, {len(classes)} classes"
    )

    head = ArcFaceHead(X_train.shape[1], len(classes), out_dim, dropout, scale, margin)
    optimizer = torch.optim.Adam(head.parameters(), lr=lr, weight_decay=weight_decay)

    best_val_acc = -1.0
    best_state = copy.deepcopy(head.state_dict())
    best_epoch = 0
    epochs_since_best = 0

    for epoch in range(epochs):
        head.train()
        optimizer.zero_grad()
        logits = head(X_train, y_train)
        loss = F.cross_entropy(logits, y_train)
        loss.backward()
        optimizer.step()

        val_acc = centroid_val_accuracy(head, y_train_str, X_train, y_val_str, X_val)
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_state = copy.deepcopy(head.state_dict())
            best_epoch = epoch
            epochs_since_best = 0
        else:
            epochs_since_best += 1

        if epoch % 10 == 0 or epoch == epochs - 1:
            print(
                f"[arcface] epoch {epoch:3d}  loss={loss.item():.4f}  val_acc={val_acc:.3f}  "
                f"(best={best_val_acc:.3f} @ epoch {best_epoch})"
            )
        if epochs_since_best >= patience:
            print(
                f"[arcface] early stop at epoch {epoch} — no val improvement in {patience} epochs"
            )
            break

    head.load_state_dict(best_state)
    head.eval()
    print(
        f"[arcface] restored best checkpoint: epoch {best_epoch}, val_acc={best_val_acc:.3f}"
    )
    return head, {"best_epoch": best_epoch, "best_val_acc": round(best_val_acc, 3)}


def apply_head(head: ArcFaceHead, emb: np.ndarray) -> np.ndarray:
    head.eval()
    with torch.no_grad():
        x = F.normalize(torch.tensor(emb, dtype=torch.float32), dim=-1)
        return head.project(x).numpy().astype(np.float32)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--source", help="path to a YOLO-seg export (not needed with --from-cache)"
    )
    parser.add_argument(
        "--n-images",
        type=int,
        default=400,
        help="train-split images to collect crops from",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--backbone", default="vit_base_patch14_dinov2.lvd142m")
    parser.add_argument(
        "--from-cache",
        action="store_true",
        help="reuse results/e2e_crops_cache_train.npz",
    )
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--lr", type=float, default=0.005)
    parser.add_argument("--out-dim", type=int, default=128)
    parser.add_argument("--dropout", type=float, default=0.3)
    parser.add_argument("--weight-decay", type=float, default=1e-3)
    parser.add_argument("--val-frac", type=float, default=0.2)
    parser.add_argument("--patience", type=int, default=25)
    parser.add_argument("--scale", type=float, default=16.0)
    parser.add_argument("--margin", type=float, default=0.3)
    args = parser.parse_args()

    if args.from_cache:
        _, train_labels, train_emb, _ = load_cached_crops(TRAIN_CACHE_PATH)
    else:
        if not args.source:
            raise SystemExit("--source is required unless --from-cache is set")
        _, train_labels, train_emb, _ = collect_real_crops(
            Path(args.source).expanduser(),
            "train",
            args.n_images,
            args.seed,
            args.backbone,
            cache_path=TRAIN_CACHE_PATH,
        )

    head, train_info = train_arcface(
        train_labels,
        train_emb,
        args.epochs,
        args.lr,
        args.out_dim,
        args.dropout,
        args.weight_decay,
        args.val_frac,
        args.patience,
        args.scale,
        args.margin,
    )

    print(
        "\n[arcface] evaluating on cached TEST-split real crops (no re-run of box proposer)..."
    )
    gallery_embeddings, held_out_labels, held_out_emb, ood_emb = load_cached_crops(
        TEST_CACHE_PATH
    )

    # --- Baseline: frozen backbone (no head), same test data ---
    baseline_global = optimize_global(
        gallery_embeddings, held_out_labels, held_out_emb, ood_emb
    )
    baseline_thresholds, baseline_result = optimize_per_class(
        gallery_embeddings,
        held_out_labels,
        held_out_emb,
        ood_emb,
        init=(baseline_global["min_similarity"], baseline_global["margin_min"]),
        rounds=3,
    )
    print(f"[arcface] FROZEN baseline (per-class): {baseline_result}")

    # --- ArcFace-projected embeddings, same test data ---
    gallery_proj = {
        name: apply_head(head, emb) for name, emb in gallery_embeddings.items()
    }
    held_out_proj = apply_head(head, held_out_emb)
    ood_proj = apply_head(head, ood_emb)

    arcface_global = optimize_global(
        gallery_proj, held_out_labels, held_out_proj, ood_proj
    )
    arcface_thresholds, arcface_result = optimize_per_class(
        gallery_proj,
        held_out_labels,
        held_out_proj,
        ood_proj,
        init=(arcface_global["min_similarity"], arcface_global["margin_min"]),
        rounds=3,
    )

    print("\n=== Comparison on the SAME cached TEST real-crop data ===")
    print(f"{'':40s} {'Recall (gated)':>16s} {'Rejection':>12s}")
    print(
        f"{'frozen backbone (per-class, current prod)':40s} {baseline_result['recall_gated'] * 100:15.1f}% {baseline_result['rejection'] * 100:11.1f}%"
    )
    print(
        f"{'ArcFace head (per-class)':40s} {arcface_result['recall_gated'] * 100:15.1f}% {arcface_result['rejection'] * 100:11.1f}%"
    )

    results_dir = Path(__file__).parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    out_path = (
        results_dir
        / f"finetune_arcface_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    out_path.write_text(
        json.dumps(
            {
                "train_info": train_info,
                "hparams": {
                    "out_dim": args.out_dim,
                    "dropout": args.dropout,
                    "weight_decay": args.weight_decay,
                    "scale": args.scale,
                    "margin": args.margin,
                },
                "frozen_baseline": baseline_result,
                "arcface_result": arcface_result,
                "arcface_thresholds": {
                    c: {"min_similarity": s, "margin_min": m}
                    for c, (s, m) in arcface_thresholds.items()
                },
            },
            indent=2,
        )
        + "\n"
    )
    print(f"\n[arcface] wrote {out_path}")

    if arcface_result["score"] > baseline_result["score"]:
        torch.save(head.state_dict(), results_dir / "arcface_head.pt")
        print(
            f"[arcface] ArcFace BEATS the frozen baseline -> {results_dir / 'arcface_head.pt'} saved"
        )
    else:
        print(
            "[arcface] ArcFace does NOT beat the frozen baseline on this data — not adopting it."
        )


if __name__ == "__main__":
    main()
