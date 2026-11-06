"""Tunables. Built from a plain dict so a notebook, a YAML file and declared
ROS parameters all feed the same dataclass."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Any, Optional

from core.types import SurfaceType

# Furniture name fragment -> surface type, used when the map has no metadata.
DEFAULT_TYPE_BY_NAME: dict[str, str] = {
    "table": SurfaceType.SURFACE.value,
    "desk": SurfaceType.SURFACE.value,
    "counter": SurfaceType.SURFACE.value,
    "bar": SurfaceType.SURFACE.value,
    "shelf": SurfaceType.SHELF.value,
    "cabinet": SurfaceType.SHELF.value,
    "tv_stand": SurfaceType.SHELF.value,
    "bed": SurfaceType.LOW_SURFACE.value,
    "sofa": SurfaceType.LOW_SURFACE.value,
    "side_table": SurfaceType.LOW_SURFACE.value,
    "coffee_table": SurfaceType.LOW_SURFACE.value,
    "refrigerator": SurfaceType.APPLIANCE.value,
    "dishwasher": SurfaceType.APPLIANCE.value,
    "microwave": SurfaceType.APPLIANCE.value,
    "washing_machine": SurfaceType.APPLIANCE.value,
    "sink": SurfaceType.APPLIANCE.value,
    "trash": SurfaceType.FLOOR.value,
    "laundry_basket": SurfaceType.FLOOR.value,
}

DEFAULT_HEIGHT_BY_TYPE: dict[str, float] = {
    SurfaceType.SURFACE.value: 0.75,
    SurfaceType.SHELF.value: 1.10,
    SurfaceType.LOW_SURFACE.value: 0.45,
    SurfaceType.APPLIANCE.value: 0.90,
    SurfaceType.FLOOR.value: 0.20,
}

# Object category -> where it belongs. Drives the misplaced-object check.
DEFAULT_CATEGORY_TO_FURNITURE: dict[str, list[str]] = {
    "drink": ["kitchen", "refrigerator"],
    "food": ["kitchen", "cabinet"],
    "fruit": ["kitchen", "counter"],
    "snack": ["living_room", "coffee_table"],
    "dish": ["kitchen", "dishwasher"],
    "cutlery": ["kitchen", "dishwasher"],
    "cleaning_supply": ["kitchen", "cabinet"],
}


@dataclass
class SemanticMapConfig:
    # Observation gating.
    confidence_min: float = 0.40

    # Instance fusion: a detection joins an instance when both tests pass.
    merge_dist_m: float = 0.25
    merge_sim_min: float = 0.80

    # Detections before an object answers queries; 1 trusts a single hit.
    confirm_min_observations: int = 1
    # Looks at an object's spot without detecting it before it is forgotten.
    miss_limit: int = 4

    # Staleness.
    stale_after_s: float = 120.0

    # Retrieval. CLIP matches captions better than bare words.
    query_template: str = "a photo of a {}"
    top_k: int = 5
    # Text against image: real matches score 0.25-0.33, unrelated queries stay under 0.24.
    sim_min: float = 0.24
    # Share of the object's name in its score: 0 is appearance only, 1 is name only.
    name_weight: float = 0.5

    # Conformal candidate set: holds the right object with probability >= 1 - alpha.
    alpha: float = 0.10
    # "threshold" keeps scores above a cutoff, "aps" keeps candidates up to a softmax mass.
    set_method: str = "aps"
    aps_temperature: float = 1.0
    # Fitted by SemanticGraph.calibrate; None keeps only the top match.
    conformal_cutoff: Optional[float] = None
    # Sets up to this size are answered by CLIP; larger ones go to the VLM.
    escalate_above: int = 1
    # Crops the VLM sees at most, as one 3x3 grid.
    max_candidates: int = 9

    # How far an object may sit from a surface and still belong to it.
    max_sublocation_distance_m: float = 1.5
    # How far ahead of its tagged pose a surface is. On a real map that pose is where
    # the robot stands, about 0.8 m in front of the furniture; 0 uses the pose itself.
    surface_standoff_m: float = 0.0
    # Tagged poses that are navigation waypoints, not furniture.
    non_surface_names: tuple[str, ...] = (
        "house_entry",
        "entry",
        "entrance",
        "exit",
        "door",
        "start",
        "start_point",
    )

    # How close counts as "next to" an anchor.
    near_m: float = 1.0

    type_by_name: dict[str, str] = field(
        default_factory=lambda: dict(DEFAULT_TYPE_BY_NAME)
    )
    height_by_type: dict[str, float] = field(
        default_factory=lambda: dict(DEFAULT_HEIGHT_BY_TYPE)
    )
    default_type: str = SurfaceType.SURFACE.value

    category_to_furniture: dict[str, list[str]] = field(
        default_factory=lambda: {
            k: list(v) for k, v in DEFAULT_CATEGORY_TO_FURNITURE.items()
        }
    )

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "SemanticMapConfig":
        """Build from a partial dict, ignoring unknown keys."""
        if not data:
            return cls()
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})
