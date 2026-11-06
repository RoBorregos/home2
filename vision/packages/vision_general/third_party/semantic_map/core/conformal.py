"""Split conformal prediction over retrieval rankings. Pure numpy."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def quantile(nonconformity: Sequence[float], alpha: float) -> float:
    """Cutoff such that keeping values <= it covers a new case with probability >= 1 - alpha."""
    values = np.sort(np.asarray(nonconformity, dtype=float))
    rank = math.ceil((len(values) + 1) * (1 - alpha))
    return math.inf if rank > len(values) else float(values[rank - 1])


def mass_before(scores: np.ndarray, temperature: float) -> np.ndarray:
    """Softmax mass ranked strictly above each score; scores come best first."""
    weights = np.exp((scores - scores.max()) / temperature)
    return np.concatenate([[0.0], np.cumsum(weights / weights.sum())[:-1]])
