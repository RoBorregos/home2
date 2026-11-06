"""File loading. The only place that opens files; the core never does."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from core.config import SemanticMapConfig
from core.serialize import snapshot_from_dict, snapshot_to_dict
from core.types import Snapshot

EMBEDDINGS_NAME = "embeddings.npz"
IMAGES_NAME = "images"


def load_snapshot(path: str | Path, config: SemanticMapConfig) -> Snapshot:
    """Read a snapshot, with the crops and image embeddings beside it when they exist."""
    path = Path(path)
    with open(path) as file:
        snapshot = snapshot_from_dict(json.load(file), config)
    for obj in snapshot.objects.values():
        crop = path.parent / IMAGES_NAME / f"{obj.id}.jpg"
        obj.crop = str(crop) if crop.exists() else ""
    embeddings = path.parent / EMBEDDINGS_NAME
    if embeddings.exists():
        with np.load(embeddings) as vectors:
            for object_id in vectors.files:
                if object_id in snapshot.objects:
                    snapshot.objects[object_id].embedding = vectors[object_id]
    return snapshot


def save_snapshot(snapshot: Snapshot, path: str | Path) -> None:
    """Write a snapshot the way load_snapshot reads it: JSON, embeddings beside it.

    The embeddings file is always rewritten, even empty, so a stale one from an
    earlier save can never be loaded against these objects.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = snapshot_to_dict(snapshot)
    for raw in data["objects"]:
        raw["embedding"] = None
    with open(path, "w") as file:
        json.dump(data, file, indent=2, ensure_ascii=False)
    vectors = {
        i: o.embedding for i, o in snapshot.objects.items() if o.embedding is not None
    }
    np.savez(path.parent / EMBEDDINGS_NAME, **vectors)
