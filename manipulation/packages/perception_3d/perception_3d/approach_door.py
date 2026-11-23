"""Door lever handle from one depth image: door planes, then lever-sized blobs off them."""

from collections import Counter
from dataclasses import dataclass, replace
from typing import Optional

import numpy as np
from frida_constants.manipulation_constants import GRIPPER_MAX_APERTURE
from scipy import ndimage as ndi


@dataclass(frozen=True)
class HandleParams:
    """Segmentation gates; m in base_link (x forward, z up)."""

    max_x: float = 1.0  # points farther ahead are ignored
    y_half: float = 0.6  # handle centre at most this far left/right
    z_min: float = 0.75  # handle heights
    z_max: float = 1.25
    door_width_min: float = 0.8  # door plane width (both stiles count)
    door_height_min: float = 0.6  # visible height (the stare pose sees ~1 m)
    door_planes: int = 4  # largest vertical planes tried as the door
    door_tolerance: float = 0.01
    door_sample: int = 20000  # points the door planes are fitted on
    proud_min: float = 0.02  # off the door (cuts the lever from its neck)
    proud_max: float = 0.18  # above standoff_max + bar depth
    depth_jump: float = 0.01  # blobs are also cut at depth jumps this big
    min_pixels: int = 20
    length_min: float = 0.06  # lever width along the door
    length_max: float = 0.60  # above long pull bars (~45 cm)
    height_min: float = 0.012  # bar height
    height_max: float = GRIPPER_MAX_APERTURE  # the gripper must close on it
    standoff_min: float = 0.03  # median blob distance off the door
    standoff_max: float = 0.15  # above deep levers (~9 cm)
    face_window: float = 0.15  # door face refitted this far around the blob
    min_face_points: int = 50
    detected_window: float = 0.25  # door plane also fitted this far around a seed
    detected_radius: float = 0.08  # blob at most this far from the seed
    plane_tolerance: float = 0.006
    max_door_yaw: float = 40.0  # deg between the door normal and -x
    plane_iterations: int = 600
    plane_sample: int = 3000  # points each hypothesis is scored on
    max_residual: float = 0.006  # door face std around the blob

    def __post_init__(self):
        if self.height_max > GRIPPER_MAX_APERTURE:
            raise ValueError(
                f"height_max {self.height_max} m is over the gripper aperture"
            )


@dataclass(frozen=True)
class DoorHandle:
    """A lever handle on a vertical door, in base_link."""

    center: np.ndarray  # grasp point: bar middle on its front face
    normal: np.ndarray  # door normal, horizontal, pointing at the robot
    door_point: np.ndarray  # a point on the door face behind the lever
    length: float  # extent along the door (1-99 %)
    thickness: float  # median height of 1 cm slices
    standoff: float  # 95th pct distance off the door
    residual: float  # std of the door face inliers
    pixels: int

    @property
    def yaw(self) -> float:
        """Door heading in degrees, 0 when it squarely faces the robot."""
        return float(np.degrees(np.arctan2(-self.normal[1], -self.normal[0])))


class HandleNotFound(Exception):
    pass


def deproject(
    depth: np.ndarray, intrinsics, cam_to_base: np.ndarray, scale: float = 1.0
) -> np.ndarray:
    """(H, W, 3) base_link points of an optical-frame depth image (m), NaN without depth."""
    depth = np.where(depth > 0, depth, np.nan) * scale
    v, u = np.mgrid[0 : depth.shape[0], 0 : depth.shape[1]]
    k = intrinsics
    cam = np.stack(
        (
            (u - k.cx) * depth / k.fx,
            (v - k.cy) * depth / k.fy,
            depth,
            np.ones_like(depth),
        ),
        axis=-1,
    )
    return (cam @ cam_to_base.T)[..., :3]


def door_plane(points: np.ndarray, p: HandleParams, rng) -> Optional[tuple]:
    """Vertical plane through ``points`` facing the robot: (normal, point, inliers), or None."""
    if len(points) < 20:
        return None
    # Scored on a subset; the refit and inliers use every point.
    scored = points
    if len(points) > p.plane_sample:
        scored = points[rng.choice(len(points), p.plane_sample, replace=False)]
    best_count, best = 0, None
    for _ in range(p.plane_iterations):
        a, b = scored[rng.choice(len(scored), 2, replace=False)]
        t = (b - a)[:2]
        if np.linalg.norm(t) < 0.03:
            continue
        n = np.array([-t[1], t[0], 0.0]) / np.linalg.norm(t)
        n = -n if n[0] > 0 else n
        if -n[0] < np.cos(np.radians(p.max_door_yaw)):
            continue
        count = int((np.abs((scored - a) @ n) < p.plane_tolerance).sum())
        if count > best_count:
            best_count, best = count, (n, a)
    if best is None:
        return None
    # Least squares refit of the vertical plane on the inliers.
    q = points[np.abs((points - best[1]) @ best[0]) < p.plane_tolerance]
    centre = q[:, :2].mean(axis=0)
    _, _, vt = np.linalg.svd(q[:, :2] - centre, full_matrices=False)
    n = np.array([*vt[1], 0.0])
    n = -n if n[0] > 0 else n
    a = np.array([*centre, q[:, 2].mean()])
    return n, a, np.abs((points - a) @ n) < p.plane_tolerance


def door_planes(points: np.ndarray, p: HandleParams, rng) -> list:
    """Door-sized vertical planes facing the robot, largest first: (normal, point) each."""
    if len(points) < p.min_face_points:
        return []
    sample = points[
        rng.choice(len(points), min(len(points), p.door_sample), replace=False)
    ]
    loose = replace(p, plane_tolerance=p.door_tolerance)
    planes = []
    for _ in range(p.door_planes):
        plane = door_plane(sample, loose, rng)
        if plane is None:
            break
        normal, point, inliers = plane
        face = sample[inliers]
        width = np.ptp(np.percentile(face @ _along(normal), [1, 99]))
        height = np.ptp(np.percentile(face[:, 2], [1, 99]))
        if width >= p.door_width_min and height >= p.door_height_min:
            planes.append((normal, point))
        sample = sample[~inliers]
    return planes


def _along(normal: np.ndarray) -> np.ndarray:
    """Horizontal unit vector along a vertical door."""
    along = np.cross([0.0, 0.0, 1.0], normal)
    return along / np.linalg.norm(along)


def measure(
    points: np.ndarray, normal: np.ndarray, door_point: np.ndarray, residual: float
) -> DoorHandle:
    """Lever geometry from its points and the door face behind it."""
    offset = (points - door_point) @ normal
    along = _along(normal)
    s = points @ along
    lo, hi = np.percentile(s, [1, 99])
    slices = [
        points[(s >= b) & (s < b + 0.01), 2] for b in np.arange(lo, hi + 0.01, 0.01)
    ]
    heights = [np.ptp(z) for z in slices if len(z) >= 5]
    thickness = (
        np.median(heights) if heights else np.ptp(np.percentile(points[:, 2], [2, 98]))
    )
    standoff = float(np.percentile(offset, 95))
    # The bar's middle on its front face, not the visible faces' centroid.
    middle = (
        door_point + along * ((lo + hi) / 2 - door_point @ along) + normal * standoff
    )
    middle[2] = np.median(points[:, 2])
    return DoorHandle(
        center=middle,
        normal=normal,
        door_point=door_point,
        length=float(hi - lo),
        thickness=float(thickness),
        standoff=standoff,
        residual=float(residual),
        pixels=len(points),
    )


def _depth_edges(depth: np.ndarray, jump: float) -> np.ndarray:
    """Pixels next to a depth jump; NaN neighbours never count as a jump."""
    with np.errstate(invalid="ignore"):
        jump_v = np.abs(np.diff(depth, axis=0)) > jump
        jump_u = np.abs(np.diff(depth, axis=1)) > jump
    edges = np.zeros(depth.shape, dtype=bool)
    edges[1:] |= jump_v
    edges[:-1] |= jump_v
    edges[:, 1:] |= jump_u
    edges[:, :-1] |= jump_u
    return edges


def find_handles(
    points: np.ndarray,
    depth: np.ndarray,
    p: HandleParams = HandleParams(),
    seed: int = 0,
    rejections: Optional[Counter] = None,
    detected: Optional[np.ndarray] = None,
) -> list:
    """Lever-sized blobs standing off a door plane; with ``detected``, only those at it."""
    rng = np.random.default_rng(seed)
    rejections = Counter() if rejections is None else rejections
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(points).all(axis=-1) & (points[..., 0] <= p.max_x)
    planes = door_planes(points[ok], p, rng)
    if detected is not None:
        # Also the plane around the seed: the door may not look door-sized.
        with np.errstate(invalid="ignore"):
            window = ok & (
                np.linalg.norm(points - detected, axis=-1) < p.detected_window
            )
        if window.sum() >= p.min_face_points:
            loose = replace(p, plane_tolerance=p.door_tolerance)
            plane = door_plane(points[window], loose, rng)
            if plane is not None:
                planes.insert(0, plane[:2])
    if not planes:
        rejections["no door plane"] += 1
        return []
    edges = _depth_edges(depth, p.depth_jump)
    z = points[..., 2]
    found = []
    for door_normal, door_point in planes:
        with np.errstate(invalid="ignore"):
            offset = (points - door_point) @ door_normal
            proud = ok & ~edges & (offset > p.proud_min) & (offset < p.proud_max)
            proud &= (z > p.z_min) & (z < p.z_max)
            face = points[ok & (np.abs(offset) < p.door_tolerance)]
        along = _along(door_normal)
        face_s = face @ along
        labels, _ = ndi.label(proud, structure=np.ones((3, 3)))
        for i, box in enumerate(ndi.find_objects(labels), start=1):
            blob = labels[box] == i
            if blob.sum() < p.min_pixels:
                continue
            pts = points[box][blob]
            if (
                detected is not None
                and np.linalg.norm(pts - detected, axis=1).min() > p.detected_radius
            ):
                continue
            s = pts @ along
            length = np.ptp(np.percentile(s, [2, 98]))
            vertical = np.ptp(np.percentile(pts[:, 2], [2, 98]))
            if not (
                p.length_min < length < p.length_max
                and p.height_min < vertical < p.height_max
            ):
                continue
            # Door face refitted around the blob (doors bow, frames step).
            around = face[
                (np.abs(face_s - np.median(s)) < p.face_window)
                & (np.abs(face[:, 2] - np.median(pts[:, 2])) < p.face_window)
            ]
            plane = (
                door_plane(around, p, rng) if len(around) >= p.min_face_points else None
            )
            if plane is None:
                rejections["no door face around the blob"] += 1
                continue
            normal, face_point, inliers = plane
            standoff = np.median((pts - face_point) @ normal)
            residual = ((around[inliers] - face_point) @ normal).std()
            if not p.standoff_min < standoff < p.standoff_max:
                rejections[
                    f"stand-off outside {p.standoff_min * 1000:.0f}-{p.standoff_max * 1000:.0f} mm"
                ] += 1
            elif residual > p.max_residual:
                rejections[
                    f"door face rougher than {p.max_residual * 1000:.0f} mm"
                ] += 1
            elif abs(pts[:, 1].mean()) > p.y_half:
                rejections["beside the reach"] += 1
            else:
                handle = measure(pts, normal, face_point, residual)
                # The same lever can stand off two of the planes.
                if all(np.linalg.norm(handle.center - h.center) > 0.05 for h in found):
                    found.append(handle)
    return found


def detect_handle(
    points: np.ndarray,
    depth: np.ndarray,
    p: HandleParams = HandleParams(),
    seed: int = 0,
    detected: Optional[np.ndarray] = None,
) -> DoorHandle:
    """The lever at ``detected`` (else nearest the base), or HandleNotFound."""
    rejections = Counter()
    found = find_handles(points, depth, p, seed, rejections, detected)
    if found:
        target = np.zeros(3) if detected is None else np.asarray(detected)
        axes = slice(0, 2) if detected is None else slice(0, 3)
        return min(
            found, key=lambda h: float(np.linalg.norm((h.center - target)[axes]))
        )
    where = "standing off the door" if detected is None else "at the detected point"
    reason, count = (
        rejections.most_common(1)[0]
        if rejections
        else (f"no lever-sized blob {where}", 0)
    )
    raise HandleNotFound(f"{reason} (x{count})" if count else reason)
