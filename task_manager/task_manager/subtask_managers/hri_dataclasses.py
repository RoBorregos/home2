#!/usr/bin/env python3

"""
HRI dataclasses and enums
"""

from dataclasses import dataclass
from enum import Enum


@dataclass
class Location:
    """Location entry from the embeddings postgres service."""

    area: str
    subarea: str
    context: str = ""
    similarity: float = 0.0
    id: int = 0


@dataclass
class CommandHistory:
    """Command history entry from the embeddings postgres service."""

    id: int
    action: str
    command: str
    result: str
    status: str
    similarity: float = None
    context: str = None


@dataclass
class HandItem:
    """Hand location item from the embeddings postgres service."""

    id: int
    name: str
    description: str
    x_loc: float
    y_loc: float
    m_loc_x: float
    m_loc_y: float
    color: str


@dataclass
class FindClosestResult:
    """Bundled results and similarity scores from find_closest."""

    results: list[str]
    similarities: list[float]


class AudioStates(Enum):
    SAYING = "saying"
    LISTEN = "listening"
    IDLE = "idle"
    THINKING = "thinking"
    LOADING = "loading"

    @classmethod
    def respeaker_light(cls, state):
        if state == AudioStates.SAYING:
            return "speak"
        elif state == AudioStates.LISTEN:
            return "listen"
        elif state == AudioStates.IDLE:
            return "off"
        elif state == AudioStates.THINKING:
            return "think"
        elif state == AudioStates.LOADING:
            return "loading"
        else:
            raise ValueError(f"Unknown audio state: {state}")
