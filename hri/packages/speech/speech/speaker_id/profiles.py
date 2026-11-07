"""Enrolled speaker profiles persisted as ``<dir>/<name>.npz``.

A profile keeps a fixed ``anchor`` (mean of the enrollment embeddings, never
modified afterwards) plus a small FIFO ``pool`` of high-confidence embeddings
collected while the robot runs, so the profile can adapt to distance / room
without drifting towards an impostor.
"""

from __future__ import annotations

import json
import os
import re
import time
from dataclasses import dataclass, field

import numpy as np

from speech.speaker_id.embedder import l2_normalize


def _safe_name(name: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9_-]+", "_", name.strip().lower())
    if not clean:
        raise ValueError("empty speaker name")
    return clean


@dataclass
class SpeakerProfile:
    name: str
    anchor: np.ndarray
    enroll_embs: np.ndarray  # [N, D]
    backend: str
    enrolled_sec: float
    consistency: float
    pool: list[np.ndarray] = field(default_factory=list)
    created: float = field(default_factory=time.time)

    def score(self, emb: np.ndarray, pool_topk: int = 3) -> float:
        """Cosine score; the adaptive pool can raise it but weighs at most 50%."""
        base = float(np.dot(self.anchor, emb))
        if len(self.pool) >= 3:
            pool_scores = np.sort(np.stack(self.pool) @ emb)[::-1][:pool_topk]
            return 0.5 * base + 0.5 * max(base, float(np.mean(pool_scores)))
        return base

    def add_to_pool(self, emb: np.ndarray, pool_max: int) -> None:
        self.pool.append(l2_normalize(emb))
        if len(self.pool) > pool_max:
            self.pool.pop(0)

    def summary(self) -> dict:
        return {
            "name": self.name,
            "backend": self.backend,
            "enrolled_sec": round(self.enrolled_sec, 2),
            "consistency": round(self.consistency, 3),
            "n_enroll": int(self.enroll_embs.shape[0]),
            "pool": len(self.pool),
            "created": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.created)),
        }


class ProfileStore:
    def __init__(self, directory: str, backend: str) -> None:
        self.directory = directory
        self.backend = backend
        self.profiles: dict[str, SpeakerProfile] = {}
        os.makedirs(directory, exist_ok=True)

    def path(self, name: str) -> str:
        return os.path.join(self.directory, f"{_safe_name(name)}.npz")

    def load_all(self) -> list[str]:
        """Load every profile of the current backend. Returns skipped files."""
        self.profiles.clear()
        skipped = []
        for fname in sorted(os.listdir(self.directory)):
            if not fname.endswith(".npz"):
                continue
            try:
                prof = self._load(os.path.join(self.directory, fname))
            except Exception as exc:  # corrupted file
                skipped.append(f"{fname}: {exc}")
                continue
            if prof.backend != self.backend:
                skipped.append(f"{fname}: backend {prof.backend} != {self.backend}")
                continue
            self.profiles[prof.name] = prof
        return skipped

    def _load(self, path: str) -> SpeakerProfile:
        data = np.load(path, allow_pickle=False)
        meta = json.loads(str(data["meta"]))
        pool = list(data["pool"]) if data["pool"].size else []
        return SpeakerProfile(
            name=meta["name"],
            anchor=data["anchor"].astype(np.float32),
            enroll_embs=data["enroll_embs"].astype(np.float32),
            backend=meta["backend"],
            enrolled_sec=float(meta["enrolled_sec"]),
            consistency=float(meta["consistency"]),
            pool=[p.astype(np.float32) for p in pool],
            created=float(meta.get("created", time.time())),
        )

    def save(self, prof: SpeakerProfile) -> str:
        meta = {
            "name": prof.name,
            "backend": prof.backend,
            "enrolled_sec": prof.enrolled_sec,
            "consistency": prof.consistency,
            "created": prof.created,
        }
        pool = (
            np.stack(prof.pool)
            if prof.pool
            else np.zeros((0, prof.anchor.shape[0]), np.float32)
        )
        path = self.path(prof.name)
        tmp = path + ".tmp.npz"
        np.savez(
            tmp,
            anchor=prof.anchor,
            enroll_embs=prof.enroll_embs,
            pool=pool,
            meta=np.array(json.dumps(meta)),
        )
        os.replace(tmp, path)
        self.profiles[prof.name] = prof
        return path

    def forget(self, name: str) -> bool:
        key = _safe_name(name)
        existed = False
        for pname in list(self.profiles):
            if _safe_name(pname) == key:
                del self.profiles[pname]
                existed = True
        path = self.path(name)
        if os.path.exists(path):
            os.remove(path)
            existed = True
        return existed

    def get(self, name: str) -> SpeakerProfile | None:
        key = _safe_name(name)
        for pname, prof in self.profiles.items():
            if _safe_name(pname) == key:
                return prof
        return None
