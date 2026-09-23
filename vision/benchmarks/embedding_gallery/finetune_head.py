#!/usr/bin/env python3
"""Phase 4 (gated — only reached because Phase 1's frozen-backbone gate
failed, concentrated in cutlery fork/knife/spoon and kitchenware cup/bowl/plate
confusion; see results/benchmark_*.json). Trains a SMALL linear projection
head on top of FROZEN DINOv2-B embeddings with a batch-hard triplet loss —
the backbone itself is never touched, per the plan's "keep it only if it
beats the frozen backbone" framing.

Trains only on data/gallery_photos/ (the enrollment set) so data/held_out/,
data/hard_negatives/, data/out_of_gallery/ stay an honest, unseen evaluation
— exactly like the frozen-backbone benchmark in report.py, so the two
numbers are comparable.

Usage (this benchmark's .venv):
    python3 finetune_head.py --backbone dinov2_vitb14 --epochs 60
"""

import argparse
import itertools
import json
import sys
from pathlib import Path

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
from backbone import EmbeddingBackbone
from gallery_matcher import UNKNOWN, Gallery

from report import (
    embed_gallery_photos,
    embed_labeled_dir,
    embed_unlabeled_dir,
    DATA_DIR,
    SIM_GRID,
    MARGIN_GRID,
)

RECALL_TARGET = 0.90
REJECTION_TARGET = 0.80
HERE = Path(__file__).parent


class ProjectionHead(nn.Module):
    """A single frozen-embedding -> compact-embedding linear map. Small on
    purpose: ~690 training crops over 23 classes can't support anything
    deeper without overfitting to this exact photo set instead of learning a
    reusable "which DINOv2 directions matter for our object set" projection."""

    def __init__(self, in_dim: int, out_dim: int = 256):
        super().__init__()
        self.proj = nn.Linear(in_dim, out_dim, bias=False)
        # Near-identity-ish init (via a random orthogonal-ish start) so
        # training refines the frozen space rather than starting from noise.
        nn.init.orthogonal_(self.proj.weight)

    def forward(self, x):
        return F.normalize(self.proj(x), dim=-1)


def batch_hard_triplet_loss(
    embeddings: torch.Tensor, labels: torch.Tensor, margin: float = 0.2
) -> torch.Tensor:
    """Cosine-distance batch-hard triplet loss (Hermans et al.): per anchor,
    mine the hardest positive (same class, most distant) and hardest
    negative (different class, closest) within the batch."""
    sims = embeddings @ embeddings.T  # cosine, embeddings are already L2-normalized
    dists = 1.0 - sims

    same = labels.unsqueeze(0) == labels.unsqueeze(1)
    diff = ~same
    self_mask = torch.eye(len(labels), dtype=torch.bool, device=labels.device)
    pos_mask = same & ~self_mask

    hardest_pos = (
        torch.where(pos_mask, dists, torch.full_like(dists, -1.0)).max(dim=1).values
    )
    hardest_neg = (
        torch.where(diff, dists, torch.full_like(dists, float("inf"))).min(dim=1).values
    )

    valid = pos_mask.any(dim=1) & diff.any(dim=1)
    losses = F.relu(hardest_pos - hardest_neg + margin)[valid]
    return losses.mean() if losses.numel() else torch.tensor(0.0, requires_grad=True)


def train_head(
    gallery_embeddings: dict[str, np.ndarray], epochs: int, lr: float, out_dim: int
) -> ProjectionHead:
    labels_list = sorted(gallery_embeddings)
    label_to_id = {name: i for i, name in enumerate(labels_list)}
    X = np.concatenate([gallery_embeddings[name] for name in labels_list], axis=0)
    y = np.concatenate(
        [[label_to_id[name]] * len(gallery_embeddings[name]) for name in labels_list]
    )

    X = torch.tensor(X, dtype=torch.float32)
    X = F.normalize(X, dim=-1)
    y = torch.tensor(y, dtype=torch.long)

    head = ProjectionHead(X.shape[1], out_dim)
    optimizer = torch.optim.Adam(head.parameters(), lr=lr)

    print(
        f"[finetune] training on {len(X)} crops, {len(labels_list)} classes, {epochs} epochs"
    )
    for epoch in range(epochs):
        optimizer.zero_grad()
        emb = head(X)
        loss = batch_hard_triplet_loss(emb, y)
        loss.backward()
        optimizer.step()
        if epoch % 10 == 0 or epoch == epochs - 1:
            print(f"[finetune] epoch {epoch:3d}  triplet_loss={loss.item():.4f}")

    return head


def apply_head(head: ProjectionHead, embeddings: np.ndarray) -> np.ndarray:
    with torch.no_grad():
        x = F.normalize(torch.tensor(embeddings, dtype=torch.float32), dim=-1)
        return head(x).numpy().astype(np.float32)


def sweep_and_score(
    gallery_embeddings, held_out_labels, held_out_emb, ood_emb, ood_files
) -> dict:
    def build_gallery(min_sim, margin_min):
        manifest = {
            "objects": {
                name: {"min_similarity": min_sim, "margin_min": margin_min}
                for name in gallery_embeddings
            }
        }
        return Gallery(gallery_embeddings, manifest)

    best = None
    for min_sim, margin_min in itertools.product(SIM_GRID, MARGIN_GRID):
        gallery = build_gallery(min_sim, margin_min)
        recall = sum(
            1
            for label, (pred, _, _) in zip(
                held_out_labels, gallery.match_batch(held_out_emb)
            )
            if pred == label
        ) / len(held_out_labels)
        rejection = sum(
            1 for (pred, _, _) in gallery.match_batch(ood_emb) if pred == UNKNOWN
        ) / len(ood_files)
        both_met = recall >= RECALL_TARGET and rejection >= REJECTION_TARGET
        score = min(recall, rejection)
        candidate = {
            "min_similarity": min_sim,
            "margin_min": margin_min,
            "recall_at_1": recall,
            "unknown_rejection_rate": rejection,
            "both_targets_met": both_met,
            "score": score,
        }
        if best is None or (candidate["both_targets_met"], candidate["score"]) > (
            best["both_targets_met"],
            best["score"],
        ):
            best = candidate
    return best


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--backbone", default="vit_base_patch14_dinov2.lvd142m")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--lr", type=float, default=0.01)
    parser.add_argument("--out-dim", type=int, default=256)
    args = parser.parse_args()

    print("[finetune] embedding all crops with the frozen backbone...")
    backbone = EmbeddingBackbone(args.backbone).load()
    gallery_raw = embed_gallery_photos(backbone)
    held_out_files, held_out_labels, held_out_raw = embed_labeled_dir(
        backbone, DATA_DIR / "held_out"
    )
    hard_neg_files, hard_neg_labels, hard_neg_raw = embed_labeled_dir(
        backbone, DATA_DIR / "hard_negatives"
    )
    ood_files, ood_raw = embed_unlabeled_dir(backbone, DATA_DIR / "out_of_gallery")

    # --- Baseline: frozen backbone, no head ---
    baseline_best = sweep_and_score(
        gallery_raw, held_out_labels, held_out_raw, ood_raw, ood_files
    )
    print(f"[finetune] FROZEN baseline: {baseline_best}")

    # --- Trained head on top of the same frozen embeddings ---
    head = train_head(gallery_raw, args.epochs, args.lr, args.out_dim)

    gallery_proj = {name: apply_head(head, emb) for name, emb in gallery_raw.items()}
    held_out_proj = apply_head(head, held_out_raw)
    hard_neg_proj = apply_head(head, hard_neg_raw)
    ood_proj = apply_head(head, ood_raw)

    projected_best = sweep_and_score(
        gallery_proj, held_out_labels, held_out_proj, ood_proj, ood_files
    )
    print(f"[finetune] PROJECTED (trained head): {projected_best}")

    # Final detailed metrics at the projected head's best threshold
    manifest = {
        "objects": {
            name: {
                "min_similarity": projected_best["min_similarity"],
                "margin_min": projected_best["margin_min"],
            }
            for name in gallery_proj
        }
    }
    gallery = Gallery(gallery_proj, manifest)
    recall_preds = gallery.match_batch(held_out_proj)
    hard_neg_preds = gallery.match_batch(hard_neg_proj)
    ood_preds = gallery.match_batch(ood_proj)

    recall_at_1 = sum(
        1 for label, (p, _, _) in zip(held_out_labels, recall_preds) if p == label
    ) / len(held_out_labels)
    hard_neg_precision = sum(
        1
        for label, (p, _, _) in zip(hard_neg_labels, hard_neg_preds)
        if p == UNKNOWN or p == label
    ) / len(hard_neg_labels)
    unknown_rejection = sum(1 for (p, _, _) in ood_preds if p == UNKNOWN) / len(
        ood_preds
    )

    print("\n=== Summary ===")
    print(f"{'':20s} {'Recall@1':>10s} {'HardNeg':>10s} {'Rejection':>10s}")
    print(
        f"{'frozen (no head)':20s} {baseline_best['recall_at_1']*100:9.0f}% {'':>10s} {baseline_best['unknown_rejection_rate']*100:9.0f}%"
    )
    print(
        f"{'trained head':20s} {recall_at_1*100:9.0f}% {hard_neg_precision*100:9.0f}% {unknown_rejection*100:9.0f}%"
    )

    targets_met = recall_at_1 >= RECALL_TARGET and unknown_rejection >= REJECTION_TARGET
    print(
        f"\nGate (>=90% recall, >=80% rejection): {'PASS' if targets_met else 'FAIL'}"
    )

    if targets_met:
        torch.save(head.state_dict(), HERE / "results" / "projection_head.pt")
        (HERE / "results" / "projection_head_meta.json").write_text(
            json.dumps(
                {
                    "backbone": args.backbone,
                    "out_dim": args.out_dim,
                    "min_similarity": projected_best["min_similarity"],
                    "margin_min": projected_best["margin_min"],
                    "recall_at_1": round(recall_at_1, 3),
                    "hard_negative_precision": round(hard_neg_precision, 3),
                    "unknown_rejection_rate": round(unknown_rejection, 3),
                },
                indent=2,
            )
            + "\n"
        )
        print(
            "[finetune] GATE PASSED -> results/projection_head.pt + projection_head_meta.json"
        )


if __name__ == "__main__":
    main()
