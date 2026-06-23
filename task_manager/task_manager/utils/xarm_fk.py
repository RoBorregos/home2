"""Analytic FK for the xArm6 mounted on the FRIDA base.

All constants come from robot_description/.../xarm6.urdf (and the FRIDA mount
+ gripper_grasp_frame). Used by `align_to_centroid_height` and
`align_arm_toward_centroid` to predict the grasp frame in `base_link` without
round-tripping through MoveIt.
"""

from __future__ import annotations

import math
from typing import Callable, Optional, Tuple

import numpy as np

XARM_MOUNT_XYZ = (0.036105613, 0.0, 0.441)
XARM_MOUNT_YAW = math.pi / 2
J1_OFFSET = (0.0, 0.0, 0.267)
J3_OFFSET = (0.0535, -0.2845, 0.0)
J4_OFFSET = (0.0775, 0.3425, 0.0)
J6_OFFSET = (0.076, 0.097, 0.0)
GRASP_YAW = -math.pi / 4

# Soft limits stay inside the xArm6 hardware range so the solver never
# commands into a hard stop. Hardware: j2 ∈ [-118°, +120°], j5 ∈ [-97°, +178°].
J2_MIN = math.radians(-110)
J2_MAX = math.radians(90)
J5_MIN = math.radians(-90)
J5_MAX = math.radians(115)


def _T_trans(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def _T_rot(axis: str, a: float) -> np.ndarray:
    T = np.eye(4)
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        T[1, 1], T[1, 2], T[2, 1], T[2, 2] = c, -s, s, c
    elif axis == "y":
        T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, s, -s, c
    else:
        T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    return T


def fk_grasp_frame(j1: float, j2: float, j3: float, j4: float, j5: float, j6: float) -> np.ndarray:
    """4x4 transform of `gripper_grasp_frame` expressed in `base_link`."""
    T = _T_trans(*XARM_MOUNT_XYZ) @ _T_rot("z", XARM_MOUNT_YAW)
    T = T @ _T_trans(*J1_OFFSET) @ _T_rot("z", j1)
    T = T @ _T_rot("x", -math.pi / 2) @ _T_rot("z", j2)
    T = T @ _T_trans(*J3_OFFSET) @ _T_rot("z", j3)
    T = T @ _T_trans(*J4_OFFSET) @ _T_rot("x", -math.pi / 2) @ _T_rot("z", j4)
    T = T @ _T_rot("x", math.pi / 2) @ _T_rot("z", j5)
    T = T @ _T_trans(*J6_OFFSET) @ _T_rot("x", -math.pi / 2) @ _T_rot("z", j6)
    T = T @ _T_rot("z", GRASP_YAW)
    return T


def bisect(
    fn: Callable[[float], float],
    lo: float,
    hi: float,
    target: float = 0.0,
    tol: float = 1e-4,
    max_iter: int = 80,
) -> Optional[float]:
    """Root of fn(x) - target in [lo, hi]. Returns midpoint if no sign change."""
    flo = fn(lo) - target
    fhi = fn(hi) - target
    if flo * fhi > 0:
        return None
    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        fmid = fn(mid) - target
        if abs(fmid) < tol:
            return mid
        if flo * fmid < 0:
            hi = mid
            fhi = fmid
        else:
            lo = mid
            flo = fmid
    return 0.5 * (lo + hi)


def _solve_j5_horizontal(j1: float, j2: float, j3: float, j4: float, j6: float) -> Optional[float]:
    """j5 such that the gripper approach axis is horizontal AND forward.

    T[2,2] == 0 has two roots in [J5_MIN, J5_MAX]; the forward one (T[0,2] > 0)
    is selected via a discrete sweep, then refined with a tight bisection so
    the refinement cannot jump branches.
    """

    def fk(j5: float) -> np.ndarray:
        return fk_grasp_frame(j1, j2, j3, j4, j5, j6)

    best_j5 = None
    best_score = -math.inf
    for v in np.linspace(J5_MIN, J5_MAX, 121):
        T = fk(float(v))
        if T[0, 2] <= 0.0:
            continue
        score = -abs(T[2, 2])
        if score > best_score:
            best_score = score
            best_j5 = float(v)

    if best_j5 is None:
        return None

    window = math.radians(20.0)
    lo = max(J5_MIN, best_j5 - window)
    hi = min(J5_MAX, best_j5 + window)
    refined = bisect(lambda v: fk(v)[2, 2], lo, hi, target=0.0)
    if refined is not None and fk(refined)[0, 2] > 0.0:
        best_j5 = refined

    return float(max(J5_MIN, min(J5_MAX, best_j5)))


def _solve_j2_for_height(
    j1: float,
    j3: float,
    j4: float,
    j5: float,
    j6: float,
    target_z: float,
) -> Tuple[Optional[float], bool]:
    """j2 such that gripper_grasp_frame.z == target_z.

    With j3=-90° (typical lock), wrist-z vs j2 is unimodal with a peak around
    j2≈-45°, so a target below the peak has two roots. We always pick the
    higher j2 (arm extended forward, not folded back).
    """

    def f(j2: float) -> float:
        return fk_grasp_frame(j1, j2, j3, j4, j5, j6)[2, 3]

    sweep = np.linspace(J2_MIN, J2_MAX, 60)
    zs = [f(v) for v in sweep]
    brackets = [
        (float(sweep[i]), float(sweep[i + 1]))
        for i in range(len(sweep) - 1)
        if (zs[i] - target_z) * (zs[i + 1] - target_z) <= 0
    ]
    if not brackets:
        return None, False

    bracket = max(brackets, key=lambda b: b[1])
    j2 = bisect(f, bracket[0], bracket[1], target=target_z)
    if j2 is None:
        j2 = 0.5 * (bracket[0] + bracket[1])
    return float(max(J2_MIN, min(J2_MAX, j2))), True


def solve_j2_j5_for_height(
    j1: float,
    j3: float,
    j4: float,
    j6: float,
    target_z: float,
    j2_seed: float,
    j5_seed: float,
    iterations: int = 4,
    convergence_tol: float = 1e-4,
) -> Tuple[Optional[float], Optional[float], bool]:
    """Resolve (j2, j5) so the grasp frame reaches `target_z` with the
    gripper horizontal. Returns (None, None, False) if no forward-pointing
    horizontal solution exists at this height.
    """
    j2, j5 = j2_seed, j5_seed
    reached = False
    for _ in range(iterations):
        j5_new = _solve_j5_horizontal(j1, j2, j3, j4, j6)
        if j5_new is None:
            return None, None, False
        j2_new, reached = _solve_j2_for_height(j1, j3, j4, j5_new, j6, target_z)
        if j2_new is None:
            return None, None, False
        if abs(j2_new - j2) < convergence_tol and abs(j5_new - j5) < convergence_tol:
            j2, j5 = j2_new, j5_new
            break
        j2, j5 = j2_new, j5_new
    return j2, j5, reached


# Joint2 axis origin in base_link, derived from the URDF chain
# base_link -> xarm_base (0.036, 0, 0.441) -> link1 (+z 0.267).
SHOULDER_IN_BASE = np.array([0.036105613, 0.0, 0.441 + 0.267])


def _best_alignment(
    fk_axis,
    fk_dir_fn,
    target_unit: np.ndarray,
    lo: float,
    hi: float,
    sweep_points: int,
):
    """Sweep an angle in [lo, hi] and return the value that maximises the
    cosine between `fk_dir_fn(angle)` and `target_unit`.
    """
    best_value: Optional[float] = None
    best_cos = -math.inf
    best_dir = None
    extra = 0.0
    for v in np.linspace(lo, hi, sweep_points):
        direction, magnitude = fk_dir_fn(float(v))
        if direction is None:
            continue
        cos = float(np.dot(direction, target_unit))
        if cos > best_cos:
            best_cos = cos
            best_value = float(v)
            best_dir = direction
            extra = magnitude
    return best_value, best_cos, best_dir, extra


def point_arm_at_centroid(
    centroid_xyz: Tuple[float, float, float],
    j3_lock: float,
    j4_lock: float,
    j5_seed: float,
    j6_lock: float,
    j2_min: float = J2_MIN,
    j2_max: float = J2_MAX,
    j5_min: float = J5_MIN,
    j5_max: float = J5_MAX,
    sweep_points: int = 240,
    refinement_iters: int = 3,
) -> Tuple[Optional[float], Optional[float], Optional[float], dict]:
    """Solve (j1, j2, j5) so the arm reaches toward `centroid_xyz` AND the
    gripper approach axis (T[:3,2]) points at the same target. j3, j4, j6 stay
    locked at the supplied values.

    Iterates j2 ↔ j5 so the position correction from j5 (the wrist offset is
    ~12 cm from joint5) doesn't desync the arm direction. Returns
    (None, None, None, info) when no positive cosine is reachable on either
    axis.
    """
    target_vec = np.array(centroid_xyz, dtype=float) - SHOULDER_IN_BASE
    target_dist = float(np.linalg.norm(target_vec))
    if target_dist < 1e-6:
        return None, None, None, {"error": "centroid coincides with shoulder"}
    target_unit = target_vec / target_dist

    azimuth = math.atan2(target_unit[1], target_unit[0])
    j1 = -math.pi / 2 + azimuth

    def arm_dir(j2: float, j5: float):
        T = fk_grasp_frame(j1, j2, j3_lock, j4_lock, j5, j6_lock)
        grasp_vec = T[:3, 3] - SHOULDER_IN_BASE
        d = float(np.linalg.norm(grasp_vec))
        return (grasp_vec / d, d) if d > 1e-6 else (None, 0.0)

    def approach_dir(j2: float, j5: float):
        T = fk_grasp_frame(j1, j2, j3_lock, j4_lock, j5, j6_lock)
        ax = T[:3, 2]
        n = float(np.linalg.norm(ax))
        return (ax / n, 0.0) if n > 1e-6 else (None, 0.0)

    current_j5 = float(j5_seed)
    current_j2: Optional[float] = None
    arm_cos = approach_cos = -math.inf
    arm_dir_v = approach_dir_v = None
    arm_reach = 0.0

    for _ in range(max(1, refinement_iters)):
        j2, arm_cos, arm_dir_v, arm_reach = _best_alignment(
            "j2",
            lambda v: arm_dir(v, current_j5),
            target_unit,
            j2_min,
            j2_max,
            sweep_points,
        )
        if j2 is None or arm_cos < 0.0:
            return (
                None,
                None,
                None,
                {
                    "azimuth_deg": math.degrees(azimuth),
                    "target_dist": target_dist,
                    "target_unit": target_unit.tolist(),
                    "arm_cos": arm_cos,
                    "error": "no j2 gives forward arm alignment",
                },
            )
        current_j2 = j2

        j5, approach_cos, approach_dir_v, _ = _best_alignment(
            "j5",
            lambda v: approach_dir(current_j2, v),
            target_unit,
            j5_min,
            j5_max,
            sweep_points,
        )
        if j5 is None or approach_cos < 0.0:
            return (
                None,
                None,
                None,
                {
                    "azimuth_deg": math.degrees(azimuth),
                    "target_dist": target_dist,
                    "target_unit": target_unit.tolist(),
                    "arm_cos": arm_cos,
                    "approach_cos": approach_cos,
                    "error": "no j5 gives forward gripper approach",
                },
            )
        current_j5 = j5

    info = {
        "azimuth_deg": math.degrees(azimuth),
        "target_dist": target_dist,
        "target_unit": target_unit.tolist(),
        "arm_cos": arm_cos,
        "arm_dir": arm_dir_v.tolist() if arm_dir_v is not None else None,
        "arm_reach_at_best": arm_reach,
        "approach_cos": approach_cos,
        "approach_dir": approach_dir_v.tolist() if approach_dir_v is not None else None,
    }
    return float(j1), float(current_j2), float(current_j5), info
