"""What the robot has seen on each piece of furniture during a patrol."""

import json
import math
from collections import Counter
from dataclasses import dataclass, replace
from typing import Callable, NamedTuple, Optional

import numpy as np

# Two detections closer than this can be the same object
SAME_OBJECT_M = 0.15
# Minimum crop-to-crop similarity for two detections to be the same object
SAME_CROP_SIMILARITY = 0.8
# Minimum CLIP text-to-crop similarity for a match
MIN_SIMILARITY = 0.24
# How close counts as "next to" an object
NEAR_M = 1.0

PARSE_PROMPT = """Split a household robot command into the object it asks for and where to look.
Every value must be in English: translate the command first if it is in another language.
- target: the object wanted, with its attributes. Empty if it is only described by its location.
- anchor: the object or piece of furniture it is located relative to. Never a room. Empty if none.
- room: one of {rooms} when the command names a room, otherwise empty.

"bring me the red drink in the kitchen" -> target "red drink", anchor "", room "kitchen"
"lo que está al lado de la cama" -> target "", anchor "bed", room ""
"dame la taza que está sobre la mesa de la sala" -> target "cup", anchor "table", room "living_room"

Command: {text}

Answer with only one JSON object with exactly these keys, filled in, and nothing else.
Example: {{"target": "", "anchor": "", "room": ""}}"""


@dataclass
class SeenObject:
    label: str
    x: float
    y: float
    z: float
    category: str = ""
    area: str = ""
    sublocation: str = ""
    embedding: Optional[np.ndarray] = None
    source: str = ""


class Misplaced(NamedTuple):
    obj: SeenObject
    expected_area: str
    expected_sublocation: str


@dataclass
class Query:
    """A command split into what is wanted and where to look for it."""

    target: str = ""
    anchor: str = ""
    room: str = ""


def parse_prompt(text: str, rooms: list[str]) -> str:
    """LLM prompt that splits `text` into the fields parse_query reads."""
    return PARSE_PROMPT.format(rooms=rooms, text=text)


def parse_query(text: str, reply: str, rooms: list[str]) -> Query:
    """The LLM's JSON split of `text`; the whole text as the target when the reply is unusable."""
    try:
        fields = json.loads(reply[reply.index("{") : reply.rindex("}") + 1])
    except ValueError:
        return Query(target=text)
    if not isinstance(fields, dict):
        return Query(target=text)
    query = Query(**{key: str(fields.get(key) or "") for key in ("target", "anchor", "room")})
    if query.room not in rooms:
        query.room = ""
    return query if query.target or query.anchor or query.room else Query(target=text)


def same(a: SeenObject, b: SeenObject) -> bool:
    """True when two detections are views of one object."""
    if math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) > SAME_OBJECT_M:
        return False
    if a.label == b.label:
        return True
    return float(a.embedding @ b.embedding) >= SAME_CROP_SIMILARITY


def fuse(samples: list[list[SeenObject]], min_hits: int) -> list[SeenObject]:
    """One object per group of detections of the same thing, kept when detected `min_hits` times."""
    groups: list[list[SeenObject]] = []
    for sample in samples:
        for obj in sample:
            group = next((g for g in groups if same(g[0], obj)), None)
            if group is None:
                groups.append([obj])
            else:
                group.append(obj)
    return [_merge(group) for group in groups if len(group) >= min_hits]


def _merge(group: list[SeenObject]) -> SeenObject:
    """Most common label, mean position and mean crop vector of a group."""
    label = Counter(o.label for o in group).most_common(1)[0][0]
    x, y, z = np.mean([(o.x, o.y, o.z) for o in group], axis=0)
    crops = [o.embedding for o in group if o.embedding is not None]
    embedding = None
    if crops:
        mean = np.mean(crops, axis=0)
        embedding = mean / (np.linalg.norm(mean) or 1.0)
    return replace(group[0], label=label, x=float(x), y=float(y), z=float(z), embedding=embedding)


def _words(text: str) -> list[str]:
    """Lowercase words, "_" as a space, plural "s" dropped: "Dinner_tables" -> dinner table."""
    cleaned = "".join(c if c.isalnum() else " " for c in text.lower().replace("_", " "))
    return [w[:-1] if len(w) > 3 and w.endswith("s") else w for w in cleaned.split()]


def _mentions(text: str, name: str) -> bool:
    """True when every word of `name` appears in `text`, in order."""
    words, wanted = _words(text), _words(name)
    return bool(wanted) and any(
        words[i : i + len(wanted)] == wanted for i in range(len(words) - len(wanted) + 1)
    )


class SceneMemory:
    def __init__(self, default_location: dict[str, list[str]]):
        self.default_location = default_location
        self._seen: list[SeenObject] = []

    def clear(self) -> None:
        self._seen = []

    def record(self, source: str, seen: list[SeenObject]) -> None:
        """Replace what was seen from `source` with the latest look."""
        kept = [o for o in self._seen if o.source != source]
        self._seen = kept + [replace(o, source=source) for o in seen]

    def objects(self) -> list[SeenObject]:
        """Every object once, dropping repeats seen from other viewpoints."""
        unique = []
        for obj in self._seen:
            if not any(same(o, obj) for o in unique):
                unique.append(obj)
        return unique

    def misplaced(self, has_furniture: Callable[[str, str], bool]) -> list[Misplaced]:
        """Objects off their category's default furniture, when this map has that furniture."""
        found = []
        for obj in self.objects():
            expected = self.default_location.get(obj.category)
            if not expected or not obj.sublocation or not has_furniture(*expected):
                continue
            if [obj.area, obj.sublocation] != list(expected):
                found.append(Misplaced(obj, *expected))
        return found

    def select(self, query: Query, text_embedding: Optional[np.ndarray] = None) -> list[SeenObject]:
        """Objects a command means, best first; all at the anchor or room when it names no target."""
        objects = [o for o in self.objects() if query.room in ("", o.area)]
        if query.anchor:
            objects = _around(query.anchor, objects)
        if not query.target:
            return objects
        named = [o for o in objects if _mentions(query.target, o.label)]
        if named or text_embedding is None:
            return named
        scored = [
            (float(o.embedding @ text_embedding), o) for o in objects if o.embedding is not None
        ]
        scored.sort(key=lambda pair: pair[0], reverse=True)
        return [o for score, o in scored if score >= MIN_SIMILARITY]


def _around(anchor: str, objects: list[SeenObject]) -> list[SeenObject]:
    """Objects on the furniture `anchor` names, else within NEAR_M of the object it names."""
    on_furniture = [o for o in objects if o.sublocation and _mentions(o.sublocation, anchor)]
    if on_furniture:
        return on_furniture
    anchors = [o for o in objects if _mentions(anchor, o.label)]
    return [
        o
        for o in objects
        if all(o is not a for a in anchors)
        and any(math.dist((o.x, o.y), (a.x, a.y)) <= NEAR_M for a in anchors)
    ]
