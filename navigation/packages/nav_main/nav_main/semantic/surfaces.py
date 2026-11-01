"""Tagged viewpoints derived from the active map's areas JSON.

The sublocation poses in ``areas_<MAP_NAME>.json`` are robot poses (base position
plus heading facing the furniture), so each one is used directly as the
viewpoint. Surface type and dwell are inferred from the name.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Fixed arm pose for every viewpoint (XARM_CONFIGURATIONS)
ARM_POSE = "nav_pose"
DEFAULT_DWELL_S = 2.5

# type -> (target height in m, dwell seconds)
SURFACE_TYPES: dict[str, tuple[float, float]] = {
    "surface": (0.75, 2.5),
    "low_surface": (0.45, 2.5),
    "shelf": (1.10, 3.0),
    "appliance": (0.90, 2.5),
    "floor": (0.20, 2.5),
    "unknown": (0.75, DEFAULT_DWELL_S),
}

# Waypoints, not furniture: nothing to look at
NON_SURFACE_NAMES = frozenset(
    {"house_entry", "entry", "entrance", "exit", "door", "start", "start_point"}
)

# Substring -> type, first match wins ("bedside_table" before "table")
NAME_TYPE_RULES: tuple[tuple[str, str], ...] = (
    ("washing_machine", "appliance"),
    ("dishwasher", "appliance"),
    ("refrigerator", "appliance"),
    ("microwave", "appliance"),
    ("oven", "appliance"),
    ("stove", "appliance"),
    ("sink", "appliance"),
    ("bedside_table", "low_surface"),
    ("side_table", "low_surface"),
    ("coffee_table", "low_surface"),
    ("tv_stand", "shelf"),
    ("bookshelf", "shelf"),
    ("shelf", "shelf"),
    ("cabinet", "shelf"),
    ("pantry", "shelf"),
    ("laundry_basket", "floor"),
    ("waste_basket", "floor"),
    ("trash_bin", "floor"),
    ("trashcan", "floor"),
    ("trash", "floor"),
    ("basket", "floor"),
    ("bin", "floor"),
    ("bed", "low_surface"),
    ("sofa", "low_surface"),
    ("couch", "low_surface"),
    ("seats", "low_surface"),
    ("chair", "low_surface"),
    ("coat_rack", "shelf"),
    ("rack", "shelf"),
    ("hanger", "shelf"),
    ("counter", "surface"),
    ("desk", "surface"),
    ("bar", "surface"),
    ("table", "surface"),
    ("surface", "surface"),
    ("items", "surface"),
)

# Not sublocations: the room outline and nav's standoff pose
NON_SUBLOCATION_KEYS = frozenset({"polygon", "safe_place"})


def classify(name: str) -> str:
    """Surface type for a sublocation name.

    Returns ``"waypoint"`` for entries that are navigation points rather than
    furniture, and ``"unknown"`` when no rule matches.
    """
    key = name.strip().lower()
    if key in NON_SURFACE_NAMES:
        return "waypoint"
    for token, surface_type in NAME_TYPE_RULES:
        if token in key:
            return surface_type
    return "unknown"


@dataclass
class Viewpoint:
    """A tagged robot pose in front of one piece of furniture."""

    area: str
    name: str
    x: float
    y: float
    qz: float
    qw: float
    surface_type: str = "unknown"
    arm_pose: str = ARM_POSE
    dwell_s: float = DEFAULT_DWELL_S

    @property
    def yaw(self) -> float:
        """Heading in radians (the tagger only ever stores a yaw rotation)."""
        return 2.0 * math.atan2(self.qz, self.qw)

    @property
    def key(self) -> str:
        return f"{self.area}/{self.name}"

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)


def load_viewpoints(
    areas_data: dict,
    areas: list[str] | None = None,
    require_polygon: bool = True,
    include_waypoints: bool = False,
) -> list[Viewpoint]:
    """Every tagged sublocation of `areas_data` as a :class:`Viewpoint`.

    `areas` filters by area name (``None`` = all). Entries that are navigation
    waypoints rather than furniture (``house_entry`` and friends) are skipped
    unless `include_waypoints`. With `require_polygon`, areas
    without a ``"polygon"`` are skipped: ``start_area``, ``start_location``,
    ``entrance``, ``exit`` and ``inspection_point`` are referee waypoints, not
    rooms, and patrolling them wastes time.
    """
    wanted = {a.strip().lower() for a in areas} if areas else None
    out: list[Viewpoint] = []

    for area, data in (areas_data or {}).items():
        if not isinstance(data, dict):
            continue
        if wanted is not None and area.strip().lower() not in wanted:
            continue
        if require_polygon and not data.get("polygon"):
            continue

        for name, pose in data.items():
            if name in NON_SUBLOCATION_KEYS:
                continue
            if not isinstance(pose, (list, tuple)) or len(pose) < 7:
                continue
            try:
                x, y = float(pose[0]), float(pose[1])
                qz, qw = float(pose[5]), float(pose[6])
            except (TypeError, ValueError):
                continue

            surface_type = classify(name)
            if surface_type == "waypoint" and not include_waypoints:
                continue
            _, dwell = SURFACE_TYPES.get(
                surface_type, SURFACE_TYPES["unknown"]
            )
            out.append(
                Viewpoint(
                    area=area,
                    name=name,
                    x=x,
                    y=y,
                    qz=qz,
                    qw=qw,
                    surface_type=surface_type,
                    dwell_s=dwell,
                )
            )
    return out
