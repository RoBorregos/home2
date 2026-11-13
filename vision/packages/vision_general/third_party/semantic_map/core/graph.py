"""SemanticGraph: all the logic, with no ROS and no I/O.

Two ways in: `from_snapshot` loads a graph that was already built, `load_layout`
plus `observe` builds it online. Not thread-safe on purpose — the ROS adapter
serializes access with a single mutually-exclusive callback group.
"""

from __future__ import annotations

import math
from dataclasses import astuple
from typing import Any, Optional

import numpy as np

from core import conformal
from core.config import SemanticMapConfig
from core.serialize import rooms_from_layout
from core.types import (
    Answer,
    Discrepancy,
    Observation,
    ObjectInstance,
    ObjectMatch,
    ParsedQuery,
    Point,
    PointLocation,
    Room,
    Snapshot,
    StaleSurface,
    Surface,
    SurfaceType,
)
from ports import LLM, VLM, ClipEncoder, Clock, LLMUnavailable

PARSE_PROMPT = """Split a household robot command into the object it asks for and where to look.
Every value must be in English: translate the command first if it is in another language.
- target: the object wanted, with its attributes. Empty if it is only described by its location.
- anchor: the object or piece of furniture it is located relative to. Never a room. Empty if none.
- relation: "on" or "next_to" when there is an anchor, otherwise empty.
- room: one of {rooms} when the command names a room, otherwise empty.

"bring me the red drink in the kitchen" -> target "red drink", anchor "", relation "", room "kitchen"
"lo que está al lado de la cama" -> target "", anchor "bed", relation "next_to", room ""
"dame la taza que está sobre la mesa de la sala" -> target "cup", anchor "table", relation "on", room "living_room"

Command: {text}"""

PARSE_SCHEMA = {
    "type": "object",
    "properties": {
        "target": {"type": "string"},
        "anchor": {"type": "string"},
        "relation": {"type": "string", "enum": ["", "on", "next_to"]},
        "room": {"type": "string"},
    },
    "required": ["target", "anchor", "relation", "room"],
}

VERIFY_PROMPT = """The image is a grid of {n} photos, each with its number in a yellow box at its top-left corner.
A person tells a robot: "{text}".
Answer the number of the photo that best matches the object the person wants, or "" if none could match."""


# Added to the score of an object whose label the query names outright
EXACT_LABEL_BONUS = 100.0


class SemanticGraph:
    """A three-layer map of the house: rooms, furniture, and the objects on them."""

    def __init__(
        self,
        config: Optional[SemanticMapConfig] = None,
        encoder: Optional[ClipEncoder] = None,
        llm: Optional[LLM] = None,
        clock: Optional[Clock] = None,
        vlm: Optional[VLM] = None,
    ) -> None:
        self.config = config or SemanticMapConfig()
        self.encoder = encoder
        self.llm = llm
        self.clock = clock
        self.vlm = vlm

        self.rooms: dict[str, Room] = {}
        self.objects: dict[str, ObjectInstance] = {}
        self.map_name: str = ""
        self.t_capture: float = 0.0
        self._text_vectors: dict[str, Any] = {}
        # Same command, same parse: clear it to measure LLM variability.
        self.parse_cache: dict[str, ParsedQuery] = {}
        self._last_id = 0

    # --- construction ---

    @classmethod
    def from_snapshot(cls, snapshot: Snapshot, **adapters: Any) -> "SemanticGraph":
        """Load a captured graph, embedding whatever the snapshot left empty."""
        graph = cls(**adapters)
        if graph.encoder and snapshot.embedding_dim not in (0, graph.encoder.dim):
            raise ValueError(
                f"snapshot embeddings are {snapshot.embedding_dim}-d, "
                f"the encoder is {graph.encoder.dim}-d"
            )
        graph.rooms, graph.objects = snapshot.rooms, snapshot.objects
        graph.map_name, graph.t_capture = snapshot.map_name, snapshot.t_capture
        graph._embed_missing()
        return graph

    def snapshot(self) -> Snapshot:
        """The graph as it stands now, captured at the clock's time. Shares state, no copy."""
        dims = {
            len(o.embedding) for o in self.objects.values() if o.embedding is not None
        }
        if self.encoder is not None:
            dims.add(self.encoder.dim)
        if len(dims) > 1:
            raise ValueError(f"objects carry embeddings of mixed sizes {sorted(dims)}")
        return Snapshot(
            rooms=self.rooms,
            objects=self.objects,
            t_capture=self._now(),
            map_name=self.map_name,
            embedding_dim=dims.pop() if dims else 0,
        )

    def load_layout(
        self, areas: dict[str, Any], meta: Optional[dict[str, Any]] = None
    ) -> None:
        """Load the fixed layer from a raw areas.json dict."""
        self.rooms = rooms_from_layout(areas, self.config, meta)

    # --- online input ---

    def observe(self, obs: Observation) -> Optional[str]:
        """Fuse one detection. Returns the instance id, or None if gated out.

        The position is a confidence-weighted mean, the label is the most confident
        one seen, and a weak detection never lowers the confidence.
        """
        if obs.confidence < self.config.confidence_min:
            return None
        # First, so an encoder failure leaves the graph as it was.
        text_vector = None if obs.embedding is not None else self._embed(obs.label)
        same = [o for o in self.objects.values() if self._same(o, obs)]
        obj = min(same, key=lambda o: _distance(o.position, obs.position), default=None)
        if obj is None:
            obj = ObjectInstance(
                id=self._new_id(),
                label=obs.label,
                position=obs.position,
                first_seen=obs.stamp,
                observation_count=0,
            )
            self.objects[obj.id] = obj
        if obs.confidence > obj.confidence:
            obj.label, obj.category = obs.label, obs.category or obj.category
        else:
            obj.category = obj.category or obs.category
        self._fuse_position(obj, obs)
        self._fuse_embedding(obj, obs, text_vector)
        where = self.area_for_point(*astuple(obj.position))
        obj.area, obj.sublocation = where.area, where.sublocation
        obj.confidence, obj.last_seen = max(obj.confidence, obs.confidence), obs.stamp
        obj.observation_count += 1
        obj.misses = 0
        return obj.id

    def report_unseen(self, object_ids: list[str]) -> list[str]:
        """Count a look at each object's spot that did not detect it.

        The caller decides what should have been visible (see core.visibility).
        Returns the ids forgotten after `miss_limit` looks in a row.
        """
        forgotten = []
        for object_id in object_ids:
            obj = self.objects.get(object_id)
            if obj is None:
                continue
            obj.misses += 1
            if obj.misses >= self.config.miss_limit:
                del self.objects[object_id]
                forgotten.append(object_id)
        return forgotten

    def reset_objects(self) -> None:
        """Forget every object, keeping the map. Ids are not reused."""
        self.objects.clear()

    def mark_scanned(self, area: str, sublocation: str) -> None:
        """Record that the robot just looked at this surface from close enough."""
        self.rooms[area].surfaces[sublocation].last_scanned = self._now()

    # --- geometry ---

    def area_for_point(self, x: float, y: float, z: float = 0.0) -> PointLocation:
        """Resolve a point to a room and its nearest surface.

        Returns an empty area when the point falls outside every polygon by more
        than `room_margin_m`: admitting it is unknown beats naming the wrong room.
        """
        room = next((r for r in self.rooms.values() if _inside(r.polygon, x, y)), None)
        if room is None:
            room = self._nearest_room(x, y)
        if room is None:
            return PointLocation()
        where = PointLocation(area=room.name, in_house=True)
        # Something lying on the floor belongs to a floor surface, never to furniture.
        on_floor = z < self.config.height_by_type[SurfaceType.FLOOR.value]
        for surface in room.surfaces.values():
            sx, sy = self._surface_xy(surface)
            distance = math.hypot(sx - x, sy - y)
            if (
                distance < min(where.distance_m, self.config.max_sublocation_distance_m)
                and (surface.type is SurfaceType.FLOOR) == on_floor
            ):
                where.sublocation, where.distance_m = surface.name, distance
        return where

    # --- queries ---

    def known_objects(self) -> list[ObjectInstance]:
        """The objects seen often enough to answer queries."""
        return [
            o
            for o in self.objects.values()
            if o.observation_count >= self.config.confirm_min_observations
        ]

    def objects_at(self, area: str, sublocation: str = "") -> list[ObjectInstance]:
        return [
            o
            for o in self.known_objects()
            if o.area == area and sublocation in ("", o.sublocation)
        ]

    def misplaced(self) -> list[Discrepancy]:
        """Objects whose category does not match the surface they were found on.

        Says nothing about an object with no known surface, or whose expected
        surface this map does not have: neither is evidence of it being misplaced.
        """
        found = []
        for o in self.known_objects():
            expected = self.config.category_to_furniture.get(o.category)
            if not expected or not o.sublocation or not self._has_surface(*expected):
                continue
            if [o.area, o.sublocation] != list(expected):
                found.append(
                    Discrepancy(o.id, o.label, o.area, o.sublocation, *expected)
                )
        return found

    def stale(self, older_than_s: Optional[float] = None) -> list[StaleSurface]:
        """Surfaces not looked at recently, oldest first; never seen comes first."""
        limit = self.config.stale_after_s if older_than_s is None else older_than_s
        now = self._now()
        ages = [
            StaleSurface(
                s.area,
                s.name,
                s.last_scanned,
                math.inf if s.last_scanned is None else now - s.last_scanned,
            )
            for s in self._surfaces()
        ]
        return sorted((s for s in ages if s.age_s > limit), key=lambda s: -s.age_s)

    def find(self, query: str, k: Optional[int] = None) -> list[ObjectMatch]:
        """Retrieve objects by free text; without an encoder, by exact label or category."""
        ranked = self._rank_objects(query, self.known_objects())
        return [ObjectMatch(o, score) for o, score in ranked[: k or self.config.top_k]]

    # --- commands: parse is the slow LLM call, select is pure ---

    def parse(self, text: str) -> ParsedQuery:
        """Split a command into target, anchor, relation and room.

        With no usable LLM answer, the whole command is the target. That is not
        cached, so the next try can still get a real parse.
        """
        if self.llm is None:
            return ParsedQuery(target=text)
        if text in self.parse_cache:
            return self.parse_cache[text]
        rooms = [r.name for r in self.rooms.values() if r.polygon]
        try:
            answer = self.llm.complete_json(
                PARSE_PROMPT.format(rooms=rooms, text=text), PARSE_SCHEMA
            )
        except LLMUnavailable:
            return ParsedQuery(target=text)
        answer = _unwrap_schema_echo(answer, PARSE_SCHEMA)
        parsed = ParsedQuery(
            **{k: answer.get(k, "") for k in PARSE_SCHEMA["properties"]}
        )
        if parsed.room not in rooms:
            parsed.room = ""
        self.parse_cache[text] = parsed
        return parsed

    def select(
        self, parsed: ParsedQuery, k: Optional[int] = None, gated: bool = True
    ) -> list[ObjectMatch]:
        """Resolve a parsed command to objects.

        Room and relation are settled by geometry. The anchor is matched by
        name; the target by appearance and name together. Ungated, no target
        match is dropped for a low image similarity.
        """
        objects = [o for o in self.known_objects() if parsed.room in ("", o.area)]
        if parsed.anchor:
            surfaces = [s for s in self._surfaces() if parsed.room in ("", s.area)]
            anchors = self._rank(parsed.anchor, surfaces + objects)
            if not anchors:
                return []
            anchor = anchors[0][0]
            objects = [
                o
                for o in objects
                if o is not anchor and self._related(o, anchor, parsed.relation)
            ]
        if not parsed.target:
            return [ObjectMatch(o, 1.0) for o in objects]
        ranked = self._rank_objects(parsed.target, objects, gated)
        return [ObjectMatch(o, score) for o, score in ranked[: k or self.config.top_k]]

    def resolve(self, text: str) -> list[ObjectMatch]:
        return self.select(self.parse(text))

    # --- cascade: CLIP narrows to a calibrated set, the VLM only breaks ties ---

    def answer(self, text: str, parsed: Optional[ParsedQuery] = None) -> Answer:
        """Resolve a command to one object, candidates to ask about, or nothing.

        Pass `parsed` when the slow LLM parse already ran outside the caller's lock.
        """
        parsed = parsed or self.parse(text)
        if not parsed.target:
            return Answer("list", self.select(parsed, len(self.objects)))
        kept = self.candidates(parsed)
        if not kept:
            return Answer("not_found")
        # A small calibrated set means CLIP is sure enough; a large one buys a VLM look.
        if len(kept) <= self.config.escalate_above or self.vlm is None:
            return Answer("found", kept[:1])
        shown = kept[: self.config.max_candidates]
        chosen = self.verify(text, shown)
        return Answer("found", [chosen], True) if chosen else Answer("ask", kept, True)

    def candidates(self, parsed: ParsedQuery) -> list[ObjectMatch]:
        """The conformal set: ranked matches the calibrated cutoff keeps.

        Ungated, so the cutoff alone decides and every right object has a finite score.
        Uncalibrated there is no cutoff, so the similarity gate decides: the top match
        if it clears it. Otherwise a command for something not in the house would
        return whatever scored best.
        """
        cutoff = self.config.conformal_cutoff
        if cutoff is None:
            return self.select(parsed, 1)
        ranked = self.select(parsed, len(self.objects), gated=False)
        return [
            m for m, value in zip(ranked, self._conformity(ranked)) if value <= cutoff
        ]

    def verify(self, text: str, kept: list[ObjectMatch]) -> Optional[ObjectMatch]:
        """Ask the VLM which crop the command means; None when it picks none."""
        numbers = [str(i) for i in range(1, len(kept) + 1)]
        schema = {
            "type": "object",
            "properties": {"image": {"type": "string", "enum": numbers + [""]}},
            "required": ["image"],
        }
        prompt = VERIFY_PROMPT.format(n=len(kept), text=text)
        chosen = self.vlm.complete_json(prompt, schema, [m.obj.crop for m in kept]).get(
            "image"
        )
        return kept[int(chosen) - 1] if chosen in numbers else None

    def calibrate(self, commands: list[tuple[str, set[str]]]) -> float:
        """Fit the candidate-set cutoff on commands paired with the ids of their right objects."""
        nonconformity = []
        for text, right in commands:
            ranked = self.select(self.parse(text), len(self.objects), gated=False)
            values = [
                v for m, v in zip(ranked, self._conformity(ranked)) if m.obj.id in right
            ]
            nonconformity.append(min(values, default=math.inf))
        self.config.conformal_cutoff = conformal.quantile(
            nonconformity, self.config.alpha
        )
        return self.config.conformal_cutoff

    # --- internals ---

    def _now(self) -> float:
        return self.clock.now() if self.clock is not None else self.t_capture

    def _surfaces(self) -> list[Surface]:
        return [s for room in self.rooms.values() for s in room.surfaces.values()]

    def _embed(self, name: str):
        """CLIP text vector of a name or query, computed once."""
        if self.encoder is None:
            return None
        if name not in self._text_vectors:
            caption = self.config.query_template.format(name.replace("_", " "))
            self._text_vectors[name] = self.encoder.encode_text(caption)
        return self._text_vectors[name]

    def _embed_missing(self) -> None:
        """Objects that arrive without an embedding get one from their label."""
        for obj in self.objects.values():
            if obj.embedding is None:
                obj.embedding = self._embed(obj.label)
                obj.embedding_from_text = obj.embedding is not None

    def _rank(self, text: str, nodes: list) -> list[tuple[Any, float]]:
        """Score surfaces or objects against a text by name, best first."""
        if self.encoder is None:
            wanted = text.lower().replace(" ", "_")
            return [
                (n, 1.0)
                for n in nodes
                if wanted in (_name(n), getattr(n, "category", None))
            ]
        query = self._embed(text)
        scored = [(n, float(self._embed(_name(n)) @ query)) for n in nodes]
        scored = [pair for pair in scored if pair[1] >= self.config.sim_min]
        return sorted(scored, key=lambda pair: -pair[1])

    def _rank_objects(
        self, text: str, objects: list, gated: bool = True
    ) -> list[tuple[Any, float]]:
        """Score objects by appearance and name together, best first.

        Image and name similarities live on different scales, so each is
        z-scored over every object in the graph before they are mixed. Only
        the image similarity decides whether an object is a match at all, when gated.
        """
        if self.encoder is None:
            return self._rank(text, objects)
        query = self._embed(text)
        everything = self.known_objects()
        if not everything:
            return []
        image = np.array([o.embedding @ query for o in everything])
        name = np.array([self._embed(o.label) @ query for o in everything])
        weight = self.config.name_weight
        fused = (1 - weight) * _z(image) + weight * _z(name)
        wanted = {o.id for o in objects}
        # An object whose label the text names outright is a match whatever its crop
        # looks like, and outranks every object that only resembles the words.
        scored = [
            (o, float(score) + (EXACT_LABEL_BONUS if exact else 0.0))
            for o, similarity, score in zip(everything, image, fused)
            for exact in (_names_label(text, o.label),)
            if o.id in wanted
            and (not gated or exact or similarity >= self.config.sim_min)
        ]
        return sorted(scored, key=lambda pair: -pair[1])

    def _conformity(self, ranked: list[ObjectMatch]) -> np.ndarray:
        """Per-candidate value; the set keeps candidates at or below the cutoff."""
        scores = np.array([m.score for m in ranked])
        if self.config.set_method == "aps" and len(scores):
            return conformal.mass_before(scores, self.config.aps_temperature)
        return -scores

    def _related(self, obj: ObjectInstance, anchor: Any, relation: str) -> bool:
        """True when the object is on the anchor surface or, unless "on", near the anchor."""
        if isinstance(anchor, Surface):
            if (obj.area, obj.sublocation) == (anchor.area, anchor.name):
                return True
            if relation == "on":
                return False
            x, y = self._surface_xy(anchor)
        else:
            x, y = anchor.position.x, anchor.position.y
        return math.hypot(obj.position.x - x, obj.position.y - y) <= self.config.near_m

    def _same(self, obj: ObjectInstance, obs: Observation) -> bool:
        """True when a detection is another view of an instance already in the graph."""
        if _distance(obj.position, obs.position) > self.config.merge_dist_m:
            return False
        # A label's text vector scores far below merge_sim_min against any crop, even
        # of that very object, so appearance only compares crop against crop.
        if obj.embedding is None or obs.embedding is None or obj.embedding_from_text:
            return obj.label == obs.label
        return float(obj.embedding @ obs.embedding) >= self.config.merge_sim_min

    def _new_id(self) -> str:
        """An id no object has had in this run, so a forgotten object's id is never reused."""
        while True:
            self._last_id += 1
            candidate = f"obj_{self._last_id:03d}"
            if candidate not in self.objects:
                return candidate

    def _fuse_position(self, obj: ObjectInstance, obs: Observation) -> None:
        """Average the detection in by confidence."""
        # Objects loaded from an older snapshot have no weight yet.
        weight = obj.position_weight or obj.confidence * obj.observation_count
        total = weight + obs.confidence
        obj.position = Point(
            *(
                (weight * old + obs.confidence * new) / total
                for old, new in zip(astuple(obj.position), astuple(obs.position))
            )
        )
        obj.position_weight = total

    def _fuse_embedding(
        self, obj: ObjectInstance, obs: Observation, text_vector: Optional[np.ndarray]
    ) -> None:
        """Keep the mean crop embedding; the label's text only until a crop arrives."""
        if obs.embedding is None:
            if obj.embedding is None:
                obj.embedding, obj.embedding_from_text = (
                    text_vector,
                    text_vector is not None,
                )
            return
        if obj.embedding is None or obj.embedding_from_text:
            obj.embedding, obj.embedding_from_text = obs.embedding, False
            return
        mean = obj.embedding * obj.observation_count + obs.embedding
        obj.embedding = mean / np.linalg.norm(mean)

    def _surface_xy(self, surface: Surface) -> tuple[float, float]:
        """Where the furniture is: `surface_standoff_m` ahead of the tagged pose."""
        pose, standoff = surface.pose, self.config.surface_standoff_m
        yaw = 2.0 * math.atan2(pose.qz, pose.qw)
        return pose.x + standoff * math.cos(yaw), pose.y + standoff * math.sin(yaw)

    def _nearest_room(self, x: float, y: float) -> Optional[Room]:
        """The room whose outline is nearest, if within `room_margin_m`."""
        margin = self.config.room_margin_m
        if margin <= 0.0:
            return None
        rooms = [
            (_edge_distance(r.polygon, x, y), r)
            for r in self.rooms.values()
            if len(r.polygon) >= 3
        ]
        distance, room = min(rooms, key=lambda pair: pair[0], default=(math.inf, None))
        return room if distance <= margin else None

    def _has_surface(self, area: str, sublocation: str) -> bool:
        room = self.rooms.get(area)
        return room is not None and sublocation in room.surfaces


def _words(text: str) -> list[str]:
    """Lowercase words without punctuation, plural "s" dropped: "Trash bins." -> trash bin."""
    cleaned = "".join(c if c.isalnum() else " " for c in text.lower().replace("_", " "))
    return [w[:-1] if len(w) > 3 and w.endswith("s") else w for w in cleaned.split()]


def _names_label(text: str, label: str) -> bool:
    """True when every word of the label appears in the text, in order."""
    words, wanted = _words(text), _words(label)
    if not wanted:
        return False
    return any(
        words[i : i + len(wanted)] == wanted
        for i in range(len(words) - len(wanted) + 1)
    )


def _unwrap_schema_echo(answer: dict, schema: dict) -> dict:
    """Small models often echo the JSON schema and put the values under "properties"."""
    keys = schema.get("properties", {})
    inner = answer.get("properties") if isinstance(answer, dict) else None
    if (
        isinstance(inner, dict)
        and not any(k in answer for k in keys)
        and any(k in inner for k in keys)
    ):
        return {k: v for k, v in inner.items() if isinstance(v, str)}
    return answer


def _distance(a: Point, b: Point) -> float:
    return math.dist(astuple(a), astuple(b))


def _z(values: np.ndarray) -> np.ndarray:
    """Standard scores; a flat signal contributes nothing."""
    return (values - values.mean()) / (values.std() or 1.0)


def _name(node: Any) -> str:
    """Objects have a label, surfaces a name."""
    return node.label if isinstance(node, ObjectInstance) else node.name


def _edge_distance(polygon: list[tuple[float, float]], x: float, y: float) -> float:
    """Distance from a point to a polygon's outline."""
    best = math.inf
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = x2 - x1, y2 - y1
        t = max(
            0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy or 1.0))
        )
        best = min(best, math.hypot(x - x1 - t * dx, y - y1 - t * dy))
    return best


def _inside(polygon: list[tuple[float, float]], x: float, y: float) -> bool:
    """Ray casting point-in-polygon."""
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside
