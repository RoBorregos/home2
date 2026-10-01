"""Tagged viewpoints derived from the active map's areas JSON.

The sublocation poses in ``areas_<MAP_NAME>.json`` are **robot** poses, not
furniture centroids: ``map_area_tagger.py:947-951`` stores the operator's click
as the base position and the drag direction as the heading, and
``nav_central.go_to_area`` sends them straight to Nav2 as goals. So a tagged
sublocation already *is* a viewpoint in front of its furniture, and patrol
planning reuses them instead of sampling a standoff ring around a centroid.

What the JSON does not carry is what kind of surface it is, which drives the arm
"stare" pose (the camera is mounted on the gripper, ``FRIDA.urdf.xacro:81``) and
how long to dwell there so vision can confirm an object. That is inferred from
the sublocation name, and can be overridden per entry with a ``<name>_meta``
key:

    "kitchen": {
      "dinner_table": [x, y, z, qx, qy, qz, qw],
      "dinner_table_meta": {"type": "surface", "arm_pose": "table_stare",
                            "dwell_s": 3.0, "height": 0.75}
    }
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

# Arm poses come from frida_constants.xarm_configurations (XARM_CONFIGURATIONS).
# The mapping type -> pose is provisional until manipulation confirms what each
# stare pose actually looks at (request M1 in the plan).
DEFAULT_ARM_POSE = "front_stare"
DEFAULT_DWELL_S = 2.5

# type -> (target height in m, arm pose, dwell seconds)
SURFACE_TYPES: dict[str, tuple[float, str, float]] = {
    "surface": (0.75, "table_stare", 2.5),
    "low_surface": (0.45, "table_stare", 2.5),
    "shelf": (1.10, "front_stare", 3.0),
    "appliance": (0.90, "front_stare", 2.5),
    "floor": (0.20, "flat_stare", 2.5),
    "unknown": (0.75, DEFAULT_ARM_POSE, DEFAULT_DWELL_S),
}

# Sublocations that are navigation waypoints, not furniture: a door or an entry
# has no surface to stare at, so patrolling it only burns time.
NON_SURFACE_NAMES = frozenset(
    {"house_entry", "entry", "entrance", "exit", "door", "start", "start_point"}
)

# Substring -> type. Order matters: the first match wins, so put the specific
# names before the generic ones ("bedside_table" before "table").
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

# Not placement sublocations: the room outline and nav's own standoff pose.
NON_SUBLOCATION_KEYS = frozenset({"polygon", "safe_place"})
META_SUFFIX = "_meta"


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
    arm_pose: str = DEFAULT_ARM_POSE
    dwell_s: float = DEFAULT_DWELL_S
    target_height: float = 0.75
    meta: dict = field(default_factory=dict)

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
            if name in NON_SUBLOCATION_KEYS or name.endswith(META_SUFFIX):
                continue
            if not isinstance(pose, (list, tuple)) or len(pose) < 7:
                continue
            try:
                x, y = float(pose[0]), float(pose[1])
                qz, qw = float(pose[5]), float(pose[6])
            except (TypeError, ValueError):
                continue

            meta = data.get(f"{name}{META_SUFFIX}") or {}
            surface_type = str(meta.get("type") or classify(name))
            if surface_type == "waypoint" and not include_waypoints:
                continue
            height, arm_pose, dwell = SURFACE_TYPES.get(
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
                    arm_pose=str(meta.get("arm_pose") or arm_pose),
                    dwell_s=float(meta.get("dwell_s") or dwell),
                    target_height=float(meta.get("height") or height),
                    meta=dict(meta) if isinstance(meta, dict) else {},
                )
            )
    return out


def load_viewpoints_from_file(path: str, **kwargs) -> list[Viewpoint]:
    """`load_viewpoints` on an ``areas_<MAP_NAME>.json`` file (offline testing)."""
    with open(path, "r") as handle:
        return load_viewpoints(json.load(handle), **kwargs)
