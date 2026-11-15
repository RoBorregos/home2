"""Venue furniture: SDF models, spawn surfaces and scoring zones."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory

BOARD = 0.02
PANEL = 0.02
# How far above a surface an object origin may rest and still count as on it
ON_TOLERANCE = 0.05
DOOR_MODEL = "arena_door"


def load(name: str) -> dict:
    share = get_package_share_directory("frida_task_sim")
    with open(os.path.join(share, "config", "venues", f"{name}.yaml")) as f:
        return yaml.safe_load(f)


def map_yaml(venue: dict) -> str:
    maps = os.path.join(get_package_share_directory("map_context"), "maps")
    return os.path.join(maps, venue["map_name"] + ".yaml")


def keep_wall(venue: dict):
    """Predicate for build_world: drop wall boxes centred inside a carved rectangle."""

    def keep(x_min, y_min, x_max, y_max):
        x, y = (x_min + x_max) / 2, (y_min + y_max) / 2
        return not any(c[0] <= x <= c[2] and c[1] <= y <= c[3] for c in venue["carve"])

    return keep


def _material(rgb) -> str:
    c = f"{rgb[0]} {rgb[1]} {rgb[2]} 1"
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse><specular>0.1 0.1 0.1 1</specular></material>"


def _part(name: str, pose, geometry: str, rgb) -> str:
    pose = " ".join(f"{v:.4f}" for v in pose)
    friction = (
        "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>"
    )
    return f"""        <collision name="{name}_collision"><pose>{pose} 0 0 0</pose><geometry>{geometry}</geometry>{friction}</collision>
        <visual name="{name}_visual"><pose>{pose} 0 0 0</pose><geometry>{geometry}</geometry>{_material(rgb)}</visual>
"""


def _box(name, center, size, rgb) -> str:
    return _part(
        name,
        center,
        f"<box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box>",
        rgb,
    )


def furniture_sdf(name: str, f: dict) -> str:
    """Static model of one furniture, positioned in map coordinates."""
    x, y = f["center"]
    h, rgb = f["height"], f["colour"]
    parts = ""
    if f["shape"] == "cylinder":
        geometry = (
            f"<cylinder><radius>{f['radius']}</radius><length>{h}</length></cylinder>"
        )
        parts = _part("body", (0, 0, h / 2), geometry, rgb)
    elif f["shape"] == "box":
        parts = _box("body", (0, 0, h / 2), (*f["size"], h), rgb)
    elif f["shape"] == "bin":
        r = f["radius"]
        parts = _box("bottom", (0, 0, PANEL / 2), (2 * r, 2 * r, PANEL), rgb)
        for i, (dx, dy) in enumerate(((1, 0), (-1, 0), (0, 1), (0, -1))):
            size = (PANEL, 2 * r, h) if dx else (2 * r, PANEL, h)
            parts += _box(f"wall_{i}", (dx * r, dy * r, h / 2), size, rgb)
    elif f["shape"] == "shelf":
        w, d = f["size"]
        fx, fy = f["front"]
        # Back panel on the side opposite to the open front
        back = (-fx * d / 2, -fy * d / 2, h / 2)
        parts = _box("back", back, (w if fy else PANEL, PANEL if fy else d, h), rgb)
        for i, s in enumerate((-1, 1)):
            side = (s * w / 2, 0, h / 2) if fy else (0, s * d / 2, h / 2)
            parts += _box(
                f"side_{i}", side, (PANEL, d, h) if fy else (w, PANEL, h), rgb
            )
        for i, level in enumerate(f["levels"] + [h]):
            parts += _box(f"board_{i}", (0, 0, level - BOARD / 2), (w, d, BOARD), rgb)
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name="link">
{parts}      </link>
    </model>
"""


def door_sdf(venue: dict) -> str:
    """Visual-only door panel (the lidar raycasts visuals; the base has no collisions)."""
    x, y = venue["door"]["center"]
    w, d = venue["door"]["size"]
    return f"""    <model name="{DOOR_MODEL}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0.6 0 0 0</pose>
      <link name="link">
        <visual name="visual"><geometry><box><size>{w} {d} 1.2</size></box></geometry>{_material((0.45, 0.30, 0.18))}</visual>
      </link>
    </model>
"""


def surface_height(f: dict, level: int | None = None) -> float:
    if f["shape"] == "shelf":
        return f["levels"][level or 0]
    if f["shape"] == "bin":
        return PANEL
    return f["height"]


def shelf_level(f: dict, z: float) -> int | None:
    """Index of the shelf board an object at height z rests on."""
    for i, level in enumerate(f["levels"]):
        if level - 0.03 <= z <= level + ON_TOLERANCE:
            return i
    return None


def inside(f: dict, x: float, y: float, margin: float = 0.0) -> bool:
    """True when (x, y) is over the furniture footprint."""
    cx, cy = f["center"]
    if f["shape"] in ("cylinder", "bin"):
        return math.hypot(x - cx, y - cy) <= f["radius"] + margin
    w, d = f["size"]
    return abs(x - cx) <= w / 2 + margin and abs(y - cy) <= d / 2 + margin


def location_of(venue: dict, x: float, y: float, z: float) -> tuple[str, int | None]:
    """(furniture name or "floor"/"air", shelf level) where an object origin rests."""
    for name, f in venue["furniture"].items():
        if not inside(f, x, y, margin=0.02):
            continue
        if f["shape"] == "shelf":
            level = shelf_level(f, z)
            if level is not None:
                return name, level
        elif f["shape"] == "bin":
            if z <= f["height"]:
                return name, None
        elif f["height"] - 0.03 <= z <= f["height"] + ON_TOLERANCE:
            return name, None
    return ("floor" if z < 0.05 else "air"), None
