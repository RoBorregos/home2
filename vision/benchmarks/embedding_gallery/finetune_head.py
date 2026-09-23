#!/usr/bin/env python3
"""Phase 4 (gated — only reached because Phase 1's frozen-backbone gate
failed, concentrated in cutlery fork/knife/spoon and kitchenware cup/bowl/plate
confusion; see results/benchmark_*.json). Trains a SMALL linear projection
head on top of FROZEN DINOv2-B embeddings with a batch-hard triplet loss —
the backbone itself is never touched, per the plan's "keep it only if it
beats the frozen backbone" framing.

v2 — the first attempt (see git history) trained on all of gallery_photos/
with no validation signal, so nothing caught it overfitting until the final
held_out number came back worse than the frozen baseline (53% vs 80%). This
version carves an internal train/val split OUT of gallery_photos/ (val is
never used to build the matching gallery, only to decide when to stop
training) and adds weight decay + input dropout, so overfitting shows up as
a visible drop in val accuracy DURING training instead of only at the end.

KNOWN REMAINING LIMITATION: gallery_photos/ and held_out/ still come from
the SAME RCW2026_v2 capture session (same backdrop/lighting), so even this
internal val split can't fully substitute for a real second photo session —
it catches "memorized these exact 690 crops" overfitting, not "memorized
this session's background" overfitting. Re-run against held_out/ built from
a genuinely different session once one exists, to get the number the plan's
Phase 1 gate actually wants.

Usage (this benchmark's .venv):
    python3 finetune_head.py --backbone dinov2_vitb14 --epochs 300 --patience 20
"""

import argparse
import copy
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
    """A single frozen-embedding -> compact-embedding linear map, with input
    dropout as the main regularizer. Small on purpose: even with a proper
    val split, ~500 training crops over 23 classes can't support much more
    without overfitting to this exact photo set instead of learning a
    reusable "which DINOv2 directions matter for our object set" projection.
    """

    def __init__(self, in_dim: int, out_dim: int = 128, dropout: float = 0.3):
        super().__init__()
        self.dropout = nn.Dropout(dropout)
        self.proj = nn.Linear(in_dim, out_dim, bias=False)
        nn.init.orthogonal_(self.proj.weight)

    def forward(self, x):
        return F.normalize(self.proj(self.dropout(x)), dim=-1)


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


def split_train_val(
    gallery_embeddings: dict[str, np.ndarray], val_frac: float = 0.25, seed: int = 0
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Per-class split so every class appears in both — val is held OUT of
    training and out of the matching gallery used during training; it only
    ever informs when to stop."""
    rng = np.random.default_rng(seed)
    train, val = {}, {}
    for name, emb in gallery_embeddings.items():
        n = len(emb)
        idx = rng.permutation(n)
        n_val = max(1, round(n * val_frac))
        val_idx, train_idx = idx[:n_val], idx[n_val:]
        train[name] = emb[train_idx]
        val[name] = emb[val_idx]
    return train, val


def to_xy(gallery_embeddings: dict[str, np.ndarray], label_to_id: dict[str, int]):
    labels_list = sorted(gallery_embeddings)
    X = np.concatenate([gallery_embeddings[name] for name in labels_list], axis=0)
    y = np.concatenate(
        [[label_to_id[name]] * len(gallery_embeddings[name]) for name in labels_list]
    )
    X = F.normalize(torch.tensor(X, dtype=torch.float32), dim=-1)
    y = torch.tensor(y, dtype=torch.long)
    return X, y


def centroid_val_accuracy(
    head: ProjectionHead,
    train_emb: dict[str, np.ndarray],
    val_emb: dict[str, np.ndarray],
) -> float:
    """Nearest-class-centroid accuracy on val, with centroids computed from
    TRAIN only — a cheap, interpretable proxy for "does this projection
    generalize to unseen photos of the same objects," checked every epoch."""
    head.eval()
    with torch.no_grad():
        names = sorted(train_emb)
        centroids = []
        for name in names:
            x = F.normalize(torch.tensor(train_emb[name], dtype=torch.float32), dim=-1)
            proj = head(x)
            centroids.append(F.normalize(proj.mean(dim=0, keepdim=True), dim=-1))
        C = torch.cat(centroids, dim=0)  # [K, D]

        correct, total = 0, 0
        for name in names:
            x = F.normalize(torch.tensor(val_emb[name], dtype=torch.float32), dim=-1)
            proj = head(x)
            sims = proj @ C.T
            preds = sims.argmax(dim=1)
            correct += (preds == names.index(name)).sum().item()
            total += len(val_emb[name])
    head.train()
    return correct / total if total else 0.0


def train_head(
    gallery_embeddings: dict[str, np.ndarray],
    epochs: int,
    lr: float,
    out_dim: int,
    dropout: float,
    weight_decay: float,
    val_frac: float,
    patience: int,
) -> tuple[ProjectionHead, dict]:
    train_emb, val_emb = split_train_val(gallery_embeddings, val_frac)
    labels_list = sorted(gallery_embeddings)
    label_to_id = {name: i for i, name in enumerate(labels_list)}
    X_train, y_train = to_xy(train_emb, label_to_id)

    print(
        f"[finetune] {sum(len(v) for v in train_emb.values())} train / "
        f"{sum(len(v) for v in val_emb.values())} val crops, "
        f"{len(labels_list)} classes, out_dim={out_dim}, dropout={dropout}, "
        f"weight_decay={weight_decay}, patience={patience}"
    )

    head = ProjectionHead(X_train.shape[1], out_dim, dropout)
    optimizer = torch.optim.Adam(head.parameters(), lr=lr, weight_decay=weight_decay)

    best_val_acc = -1.0
    best_state = copy.deepcopy(head.state_dict())
    best_epoch = 0
    epochs_since_best = 0

    for epoch in range(epochs):
        head.train()
        optimizer.zero_grad()
        emb = head(X_train)
        loss = batch_hard_triplet_loss(emb, y_train)
        loss.backward()
        optimizer.step()

        val_acc = centroid_val_accuracy(head, train_emb, val_emb)
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_state = copy.deepcopy(head.state_dict())
            best_epoch = epoch
            epochs_since_best = 0
        else:
            epochs_since_best += 1

        if epoch % 10 == 0 or epoch == epochs - 1:
            print(
                f"[finetune] epoch {epoch:3d}  triplet_loss={loss.item():.4f}  "
                f"val_centroid_acc={val_acc:.3f}  (best={best_val_acc:.3f} @ epoch {best_epoch})"
            )

        if epochs_since_best >= patience:
            print(
                f"[finetune] early stop at epoch {epoch} — no val improvement in {patience} epochs"
            )
            break

    head.load_state_dict(best_state)
    head.eval()
    print(
        f"[finetune] restored best checkpoint: epoch {best_epoch}, val_centroid_acc={best_val_acc:.3f}"
    )
    return head, {
        "best_epoch": best_epoch,
        "best_val_centroid_acc": round(best_val_acc, 3),
    }


def apply_head(head: ProjectionHead, embeddings: np.ndarray) -> np.ndarray:
    head.eval()
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
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--lr", type=float, default=0.01)
    parser.add_argument("--out-dim", type=int, default=128)
    parser.add_argument("--dropout", type=float, default=0.3)
    parser.add_argument("--weight-decay", type=float, default=1e-3)
    parser.add_argument("--val-frac", type=float, default=0.25)
    parser.add_argument("--patience", type=int, default=20)
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

    # --- Trained head, with an internal train/val split for early stopping ---
    head, train_info = train_head(
        gallery_raw,
        args.epochs,
        args.lr,
        args.out_dim,
        args.dropout,
        args.weight_decay,
        args.val_frac,
        args.patience,
    )

    # Final gallery uses ALL enrollment photos (train+val) through the
    # early-stopped head — val's job (deciding when to stop) is done; using
    # every photo now maximizes the production-realistic gallery.
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
        f"{'frozen (no head)':20s} {baseline_best['recall_at_1'] * 100:9.0f}% {'':>10s} "
        f"{baseline_best['unknown_rejection_rate'] * 100:9.0f}%"
    )
    print(
        f"{'trained head':20s} {recall_at_1 * 100:9.0f}% {hard_neg_precision * 100:9.0f}% "
        f"{unknown_rejection * 100:9.0f}%"
    )

    targets_met = recall_at_1 >= RECALL_TARGET and unknown_rejection >= REJECTION_TARGET
    print(
        f"\nGate (>=90% recall, >=80% rejection): {'PASS' if targets_met else 'FAIL'}"
    )
    print(
        "\nReminder: gallery_photos/ and held_out/ share one RCW2026_v2 capture "
        "session (same backdrop/lighting). This number is still optimistic "
        "relative to a genuinely independent second session — see this "
        "script's docstring."
    )

    results_dir = HERE / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    (results_dir / "finetune_head_result.json").write_text(
        json.dumps(
            {
                "backbone": args.backbone,
                "out_dim": args.out_dim,
                "dropout": args.dropout,
                "weight_decay": args.weight_decay,
                "val_frac": args.val_frac,
                "train_info": train_info,
                "frozen_baseline": {
                    "recall_at_1": round(baseline_best["recall_at_1"], 3),
                    "unknown_rejection_rate": round(
                        baseline_best["unknown_rejection_rate"], 3
                    ),
                },
                "trained_head": {
                    "recall_at_1": round(recall_at_1, 3),
                    "hard_negative_precision": round(hard_neg_precision, 3),
                    "unknown_rejection_rate": round(unknown_rejection, 3),
                    "min_similarity": projected_best["min_similarity"],
                    "margin_min": projected_best["margin_min"],
                },
                "targets_met": targets_met,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"[finetune] wrote {results_dir / 'finetune_head_result.json'}")

    if targets_met:
        torch.save(head.state_dict(), results_dir / "projection_head.pt")
        print(f"[finetune] GATE PASSED -> {results_dir / 'projection_head.pt'}")


if __name__ == "__main__":
    main()
