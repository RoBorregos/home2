"""Fase 3: lightweight local RAG over docs/ai/.

Chunks the RoBorregos architecture docs by markdown heading and retrieves the
most relevant chunks for a given error text. Embeddings are computed via a local
Ollama embedding model when reachable; otherwise retrieval degrades to keyword
overlap so the oracle still gets useful context offline.

Standalone by design: no PostgreSQL, no dependency on the HRI embeddings service
(HRI may be exactly what failed)."""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_BASE_URL = os.getenv("DIAGNOSIS_OLLAMA_URL", "http://localhost:11434/v1")
DEFAULT_EMBED_MODEL = os.getenv("DIAGNOSIS_EMBED_MODEL", "nomic-embed-text")


def _resolve_docs_ai() -> Path | None:
    for parent in Path(__file__).resolve().parents:
        cand = parent / "docs" / "ai"
        if cand.is_dir():
            return cand
    return None


@dataclass
class Chunk:
    source: str  # filename
    heading: str
    text: str
    embedding: list[float] | None = None


def _split_markdown(name: str, content: str) -> list[Chunk]:
    """Split on markdown headings; keep the heading with its body."""
    chunks: list[Chunk] = []
    heading = "(intro)"
    buf: list[str] = []

    def flush():
        body = "\n".join(buf).strip()
        if body:
            chunks.append(Chunk(source=name, heading=heading, text=f"{heading}\n{body}"))

    for line in content.splitlines():
        if re.match(r"^#{1,6}\s", line):
            flush()
            heading = line.lstrip("#").strip()
            buf = []
        else:
            buf.append(line)
    flush()
    return chunks


def _cosine(a: list[float], b: list[float]) -> float:
    try:
        import numpy as np

        va, vb = np.array(a), np.array(b)
        denom = float(np.linalg.norm(va) * np.linalg.norm(vb))
        return float(va.dot(vb) / denom) if denom else 0.0
    except Exception:
        # pure-python fallback
        dot = sum(x * y for x, y in zip(a, b))
        na = sum(x * x for x in a) ** 0.5
        nb = sum(y * y for y in b) ** 0.5
        return dot / (na * nb) if na and nb else 0.0


def _keyword_score(query: str, text: str) -> float:
    q = set(re.findall(r"[a-z_]{3,}", query.lower()))
    t = set(re.findall(r"[a-z_]{3,}", text.lower()))
    if not q:
        return 0.0
    return len(q & t) / len(q)


@dataclass
class KnowledgeBase:
    base_url: str = DEFAULT_BASE_URL
    embed_model: str = DEFAULT_EMBED_MODEL
    chunks: list[Chunk] = field(default_factory=list)
    embeddings_ready: bool = False

    def load(self) -> "KnowledgeBase":
        docs = _resolve_docs_ai()
        if docs is None:
            return self
        for md in sorted(docs.glob("*.md")):
            try:
                self.chunks.extend(_split_markdown(md.name, md.read_text()))
            except OSError:
                continue
        self._try_embed()
        return self

    def _client(self):
        from openai import OpenAI

        return OpenAI(api_key=os.getenv("OPENAI_API_KEY", "ollama"), base_url=self.base_url)

    def _embed(self, text: str) -> list[float] | None:
        try:
            resp = self._client().embeddings.create(model=self.embed_model, input=text)
            return list(resp.data[0].embedding)
        except Exception:
            return None

    def _try_embed(self):
        if not self.chunks:
            return
        # Probe once; if the embedding endpoint is down, skip (keyword fallback).
        probe = self._embed(self.chunks[0].text)
        if probe is None:
            self.embeddings_ready = False
            return
        self.chunks[0].embedding = probe
        for ch in self.chunks[1:]:
            ch.embedding = self._embed(ch.text)
        self.embeddings_ready = all(c.embedding is not None for c in self.chunks)

    def retrieve(self, query: str, k: int = 3) -> list[Chunk]:
        if not self.chunks:
            return []
        if self.embeddings_ready:
            qvec = self._embed(query)
            if qvec is not None:
                ranked = sorted(
                    self.chunks,
                    key=lambda c: _cosine(qvec, c.embedding or []),
                    reverse=True,
                )
                return ranked[:k]
        # keyword fallback
        ranked = sorted(self.chunks, key=lambda c: _keyword_score(query, c.text), reverse=True)
        return [c for c in ranked[:k] if _keyword_score(query, c.text) > 0]
