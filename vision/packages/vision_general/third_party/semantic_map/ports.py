"""What the core needs from the outside world.

The core depends only on these protocols, never on ROS, torch or the filesystem.
"""

from __future__ import annotations

from typing import Any, Protocol, Sequence, runtime_checkable

import numpy as np


class LLMUnavailable(Exception):
    """The LLM did not give a usable answer. The core degrades instead of failing."""


@runtime_checkable
class ClipEncoder(Protocol):
    """CLIP text tower. Crops are embedded by vision and arrive in observations."""

    @property
    def dim(self) -> int: ...

    def encode_text(self, text: str) -> np.ndarray: ...


@runtime_checkable
class LLM(Protocol):
    """Structured completion. BAML over Ollama on the robot."""

    def complete_json(self, prompt: str, schema: dict[str, Any]) -> dict[str, Any]: ...


@runtime_checkable
class VLM(Protocol):
    """Structured completion over object crops, shown as one grid numbered from 1."""

    def complete_json(
        self, prompt: str, schema: dict[str, Any], images: Sequence[str] = ()
    ) -> dict[str, Any]: ...


@runtime_checkable
class Clock(Protocol):
    """Epoch seconds."""

    def now(self) -> float: ...
