"""Data types for the semantic graph. No ROS messages: adapters convert.

Ordered bottom-up: geometry, vocabulary, the fixed layer read from areas.json,
the dynamic layer built from perception, query results, and the snapshot that
carries all of it across a process boundary.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Point:
    """A position in the `map` frame, in meters."""

    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class Pose:
    """A position and orientation in `map`, the 7-float layout areas.json uses."""

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


# --------------------------------------------------------------------------
# Vocabulary
# --------------------------------------------------------------------------


class SurfaceType(str, Enum):
    """How a surface holds objects, which decides the arm pose used to see it."""

    SURFACE = "surface"
    SHELF = "shelf"
    LOW_SURFACE = "low_surface"
    APPLIANCE = "appliance"
    FLOOR = "floor"


# --------------------------------------------------------------------------
# Fixed layer: read once from areas.json, never changes during a run
# --------------------------------------------------------------------------


@dataclass
class Surface:
    """A piece of furniture objects sit on, and the unit the robot patrols."""

    name: str
    area: str
    pose: Pose
    type: SurfaceType
    height_m: float
    front_yaw_rad: Optional[float] = None
    arm_pose: Optional[str] = None
    last_scanned: Optional[float] = None


@dataclass
class Room:
    """An area with a polygon. Areas without one are referee waypoints, not rooms."""

    name: str
    polygon: list[tuple[float, float]] = field(default_factory=list)
    safe_place: Optional[Pose] = None
    surfaces: dict[str, Surface] = field(default_factory=dict)


# --------------------------------------------------------------------------
# Dynamic layer: built online from perception
# --------------------------------------------------------------------------


@dataclass
class Observation:
    """One detection, already projected into `map`. The input to fusion."""

    label: str
    position: Point
    stamp: float
    confidence: float = 1.0
    category: Optional[str] = None
    embedding: Optional[np.ndarray] = None


@dataclass
class ObjectInstance:
    """One object the robot believes is in the house, fused over observations."""

    id: str
    label: str
    position: Point
    area: str = ""
    sublocation: str = ""
    category: Optional[str] = None
    confidence: float = 0.0
    embedding: Optional[np.ndarray] = None
    crop: str = ""
    first_seen: float = 0.0
    last_seen: float = 0.0
    observation_count: int = 1
    # The embedding is CLIP text of the label, not of a crop: never compare it with a crop's.
    embedding_from_text: bool = False
    # Sum of the confidences behind `position`, so new detections average in by weight.
    position_weight: float = 0.0
    # Consecutive looks at where the object should be without detecting it.
    misses: int = 0


# --------------------------------------------------------------------------
# Query results
# --------------------------------------------------------------------------


@dataclass
class ParsedQuery:
    """A command split into what is wanted and where to look for it."""

    target: str = ""
    anchor: str = ""
    relation: str = ""
    room: str = ""


@dataclass
class PointLocation:
    """Where a point sits in the house. An empty area means outside every polygon."""

    area: str = ""
    sublocation: str = ""
    distance_m: float = float("inf")
    in_house: bool = False


@dataclass
class ObjectMatch:
    """One retrieval hit; a higher score is a better match."""

    obj: ObjectInstance
    score: float


@dataclass
class Answer:
    """A resolved command: "found", "ask" between candidates, "not_found", or a "list"."""

    status: str
    matches: list[ObjectMatch] = field(default_factory=list)
    vlm_used: bool = False


@dataclass
class Discrepancy:
    """An object that is not where the house expects it to be."""

    object_id: str
    label: str
    found_area: str = ""
    found_sublocation: str = ""
    expected_area: str = ""
    expected_sublocation: str = ""


@dataclass
class StaleSurface:
    """A surface the robot has not looked at recently."""

    area: str
    name: str
    last_scanned: Optional[float]
    age_s: float


# --------------------------------------------------------------------------
# Transport
# --------------------------------------------------------------------------


@dataclass
class Snapshot:
    """The whole graph at one instant: what gets dumped, loaded and replayed."""

    rooms: dict[str, Room] = field(default_factory=dict)
    objects: dict[str, ObjectInstance] = field(default_factory=dict)
    t_capture: float = 0.0
    map_name: str = ""
    embedding_dim: int = 0
