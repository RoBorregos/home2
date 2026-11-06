"""Should the camera be seeing this point right now? Pure numpy.

Not detecting an object only means it is gone if the camera was in a position to
see it, so this is what decides that a detection's absence counts as evidence.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

# Half a side of the depth patch read around the projected pixel.
DEPTH_PATCH_RADIUS_PX = 3


def should_be_visible(
    point: tuple[float, float, float],
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    width: int,
    height: int,
    depth: Optional[np.ndarray],
    max_range_m: float,
    border_px: int = 20,
    occlusion_margin_m: float = 0.15,
) -> bool:
    """Whether a point in the optical frame (z forward) falls in clear view.

    False when behind the camera, past the detector's range, too close to the image
    border for a whole box, or hidden behind something nearer. Also False without a
    valid depth reading: when unsure, do not count it against the object.
    """
    x, y, z = point
    if z <= 0.0 or z > max_range_m or depth is None:
        return False
    u, v = int(round(fx * x / z + cx)), int(round(fy * y / z + cy))
    if not (border_px <= u < width - border_px and border_px <= v < height - border_px):
        return False
    r = DEPTH_PATCH_RADIUS_PX
    patch = depth[max(v - r, 0) : v + r + 1, max(u - r, 0) : u + r + 1]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return False
    return float(np.median(valid)) >= z - occlusion_margin_m
