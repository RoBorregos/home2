"""Forward kinematics + bisection helpers for xArm6 + FRIDA mount.

Shared between the washing-machine alignment routine in
`ManipulationTasks.align_to_centroid_height` and its integration / e2e tests.
The transforms come straight from the URDF
(robot_description/frida_description/urdf/xarm/spherized_xarm/xarm6.urdf
 + FRIDA mount + gripper_grasp_frame).
"""

from __future__ import annotations

import math
from typing import Callable, Optional, Tuple

import numpy as np

# base_link -> xarm_base: trans(0.0361, 0, 0.441), rpy=(0, 0, pi/2)
XARM_MOUNT_XYZ = (0.036105613, 0.0, 0.441)
XARM_MOUNT_YAW = math.pi / 2
# xarm_base -> link1 (joint1 axis Z): trans(0, 0, 0.267)
J1_OFFSET = (0.0, 0.0, 0.267)
# link2 -> link3 (joint3 axis Z): trans(0.0535, -0.2845, 0)
J3_OFFSET = (0.0535, -0.2845, 0.0)
# link3 -> link4: trans(0.0775, 0.3425, 0), rpy=(-pi/2, 0, 0)
J4_OFFSET = (0.0775, 0.3425, 0.0)
# link5 -> link6: trans(0.076, 0.097, 0), rpy=(-pi/2, 0, 0)
J6_OFFSET = (0.076, 0.097, 0.0)
# link_eef -> gripper_grasp_frame: rpy=(0, 0, -pi/4)
GRASP_YAW = -math.pi / 4

# Soft limits keep the solver away from hard joint stops while still covering
# the full reach needed for the washing-machine opening (~30 cm to ~110 cm).
J2_MIN = math.radians(-110)
J2_MAX = math.radians(20)
J5_MIN = math.radians(-115)
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


def _solve_j5_horizontal(j1: float, j2: float, j3: float, j4: float, j6: float) -> float:
    """j5 such that gripper approach axis is horizontal (T[2,2] == 0)."""

    def f(j5: float) -> float:
        return fk_grasp_frame(j1, j2, j3, j4, j5, j6)[2, 2]

    j5 = bisect(f, J5_MIN, J5_MAX, target=0.0)
    if j5 is None:
        sweep = np.linspace(J5_MIN, J5_MAX, 80)
        j5 = float(sweep[int(np.argmin([abs(f(v)) for v in sweep]))])
    return float(max(J5_MIN, min(J5_MAX, j5)))


def _solve_j2_for_height(
    j1: float,
    j3: float,
    j4: float,
    j5: float,
    j6: float,
    target_z: float,
) -> Tuple[float, bool]:
    """j2 such that gripper_grasp_frame.z == target_z. Returns (j2, reached)."""

    def f(j2: float) -> float:
        return fk_grasp_frame(j1, j2, j3, j4, j5, j6)[2, 3]

    sweep = np.linspace(J2_MIN, J2_MAX, 40)
    zs = [f(v) for v in sweep]
    bracket = None
    for i in range(len(sweep) - 1):
        if (zs[i] - target_z) * (zs[i + 1] - target_z) <= 0:
            bracket = (sweep[i], sweep[i + 1])
            break
    if bracket is None:
        j2 = float(sweep[int(np.argmin([abs(z - target_z) for z in zs]))])
        return float(max(J2_MIN, min(J2_MAX, j2))), False
    j2 = bisect(f, bracket[0], bracket[1], target=target_z)
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
) -> Tuple[float, float, bool]:
    """Jointly resolve (j2, j5) so the grasp frame reaches `target_z`
    with the gripper horizontal. j1, j3, j4, j6 stay locked.

    Returns (j2, j5, reached_target). `reached_target` is False when the
    requested height is outside the achievable range with those locked joints.
    """
    j2, j5 = j2_seed, j5_seed
    reached = False
    for _ in range(iterations):
        j5_new = _solve_j5_horizontal(j1, j2, j3, j4, j6)
        j2_new, reached = _solve_j2_for_height(j1, j3, j4, j5_new, j6, target_z)
        if abs(j2_new - j2) < convergence_tol and abs(j5_new - j5) < convergence_tol:
            j2, j5 = j2_new, j5_new
            break
        j2, j5 = j2_new, j5_new
    return j2, j5, reached
