"""Parsing of the on-disk dicts into typed rooms and objects. No I/O."""

from __future__ import annotations

from dataclasses import asdict, astuple, replace
from typing import Any, Optional

import numpy as np

from core.config import SemanticMapConfig
from core.types import ObjectInstance, Point, Pose, Room, Snapshot, Surface, SurfaceType

NOT_SURFACES = ("polygon", "safe_place")
SNAPSHOT_VERSION = 1


def rooms_from_layout(
    layout: dict[str, Any],
    config: SemanticMapConfig,
    meta: Optional[dict[str, Any]] = None,
    scanned: Optional[dict[str, float]] = None,
) -> dict[str, Room]:
    """Build rooms and surfaces from an areas.json dict.

    The file is tagged by hand on the robot, so entries that are not a full pose
    are skipped rather than stopping the map from loading.
    """
    meta, scanned = meta or {}, scanned or {}
    rooms = {}
    for area, entry in layout.items():
        if not isinstance(entry, dict):
            continue
        room = Room(area, [tuple(p) for p in entry.get("polygon", [])])
        if _is_pose(entry.get("safe_place")):
            room.safe_place = Pose(*entry["safe_place"])
        for name, pose in entry.items():
            if (
                name in NOT_SURFACES
                or name in config.non_surface_names
                or not _is_pose(pose)
            ):
                continue
            info = meta.get(f"{area}/{name}", {})
            kind = SurfaceType(info.get("type") or _type_from_name(name, config))
            room.surfaces[name] = Surface(
                name=name,
                area=area,
                pose=Pose(*pose),
                type=kind,
                height_m=info.get("height_m", config.height_by_type[kind.value]),
                front_yaw_rad=info.get("front_yaw_rad"),
                arm_pose=info.get("arm_pose"),
                last_scanned=scanned.get(f"{area}/{name}"),
            )
        rooms[area] = room
    return rooms


def _is_pose(value: Any) -> bool:
    """True for the 7 numbers of a Pose."""
    return (
        isinstance(value, (list, tuple))
        and len(value) == 7
        and all(isinstance(v, (int, float)) for v in value)
    )


def snapshot_from_dict(data: dict[str, Any], config: SemanticMapConfig) -> Snapshot:
    """Parse a snapshot dict; object keys match the ObjectInstance fields."""
    objects = {}
    for raw in data.get("objects", []):
        embedding = raw["embedding"]
        objects[raw["id"]] = ObjectInstance(
            **{
                **raw,
                "position": Point(*raw["position"]),
                "embedding": None
                if embedding is None
                else np.asarray(embedding, np.float32),
            }
        )
    return Snapshot(
        rooms=rooms_from_layout(
            data["layout"], config, data.get("furniture_meta"), data.get("scanned")
        ),
        objects=objects,
        t_capture=data["t_capture"],
        map_name=data.get("map_name", ""),
        embedding_dim=data.get("embedding_dim", 0),
    )


def snapshot_to_dict(snapshot: Snapshot) -> dict[str, Any]:
    """The inverse of snapshot_from_dict, with embeddings inline as lists.

    Every surface gets a furniture_meta entry, so a reload never has to guess
    a type from a name.
    """
    layout, furniture_meta, scanned = {}, {}, {}
    for room in snapshot.rooms.values():
        entry: dict[str, Any] = {}
        if room.polygon:
            entry["polygon"] = [[float(x), float(y)] for x, y in room.polygon]
        if room.safe_place is not None:
            entry["safe_place"] = _floats(room.safe_place)
        for surface in room.surfaces.values():
            key = f"{room.name}/{surface.name}"
            entry[surface.name] = _floats(surface.pose)
            info = {"type": surface.type.value, "height_m": float(surface.height_m)}
            if surface.front_yaw_rad is not None:
                info["front_yaw_rad"] = float(surface.front_yaw_rad)
            if surface.arm_pose is not None:
                info["arm_pose"] = surface.arm_pose
            furniture_meta[key] = info
            if surface.last_scanned is not None:
                scanned[key] = float(surface.last_scanned)
        layout[room.name] = entry
    return {
        "version": SNAPSHOT_VERSION,
        "map_name": snapshot.map_name,
        "t_capture": float(snapshot.t_capture),
        "embedding_dim": snapshot.embedding_dim,
        "layout": layout,
        "furniture_meta": furniture_meta,
        "scanned": scanned,
        "objects": [_object_to_dict(o) for o in snapshot.objects.values()],
    }


def _object_to_dict(obj: ObjectInstance) -> dict[str, Any]:
    """Crops are left out: they are local paths, found again beside the file on load."""
    fields = asdict(replace(obj, position=None, embedding=None))
    del fields["crop"]
    return {
        **fields,
        "position": _floats(obj.position),
        "confidence": float(obj.confidence),
        "first_seen": float(obj.first_seen),
        "last_seen": float(obj.last_seen),
        "embedding": None if obj.embedding is None else obj.embedding.tolist(),
    }


def _floats(value: Any) -> list[float]:
    """A Point or Pose as the plain float list the JSON uses."""
    return [float(v) for v in astuple(value)]


def _type_from_name(name: str, config: SemanticMapConfig) -> str:
    """Guess the surface type from its name; the longest matching fragment wins."""
    fragments = [f for f in config.type_by_name if f in name]
    return (
        config.type_by_name[max(fragments, key=len)]
        if fragments
        else config.default_type
    )
