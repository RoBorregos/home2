"""What the robot has seen on each piece of furniture during a patrol."""

import math
import re
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

# Same label closer than this is the same object seen from two viewpoints
SAME_OBJECT_M = 0.25
# Minimum CLIP text-to-crop similarity for a match
MIN_SIMILARITY = 0.24


@dataclass
class SeenObject:
    label: str
    category: str
    area: str
    sublocation: str
    x: float
    y: float
    z: float
    embedding: Optional[np.ndarray] = None
    source: str = ""


class SceneMemory:
    def __init__(self, default_location: dict[str, list[str]]):
        self.default_location = default_location
        self._seen: list[SeenObject] = []

    def clear(self) -> None:
        self._seen = []

    def record(self, source: str, seen: list[SeenObject]) -> None:
        """Replace what was seen from `source` with the latest look."""
        for obj in seen:
            obj.source = source
        self._seen = [o for o in self._seen if o.source != source] + seen

    def objects(self) -> list[SeenObject]:
        """Every object once, dropping repeats seen from other viewpoints."""
        unique = []
        for obj in self._seen:
            if not any(
                o.label == obj.label and math.dist((o.x, o.y), (obj.x, obj.y)) < SAME_OBJECT_M
                for o in unique
            ):
                unique.append(obj)
        return unique

    def misplaced(self, has_furniture: Callable[[str, str], bool]) -> list[tuple]:
        """(object, expected_area, expected_sublocation) for objects off their default furniture."""
        found = []
        for obj in self.objects():
            expected = self.default_location.get(obj.category)
            if not expected or not obj.sublocation or not has_furniture(*expected):
                continue
            if [obj.area, obj.sublocation] != list(expected):
                found.append((obj, *expected))
        return found

    def find(self, text: str, text_embedding: Optional[np.ndarray] = None) -> list[SeenObject]:
        """Objects named in `text`; else the closest CLIP match, best first."""
        text = text.lower()
        named = [
            o
            for o in self.objects()
            if re.search(rf"\b{re.escape(o.label.replace('_', ' '))}\b", text)
        ]
        if named or text_embedding is None:
            return named
        scored = [
            (float(o.embedding @ text_embedding), o)
            for o in self.objects()
            if o.embedding is not None
        ]
        scored.sort(key=lambda pair: pair[0], reverse=True)
        return [o for score, o in scored if score >= MIN_SIMILARITY]
