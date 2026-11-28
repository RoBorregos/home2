"""Compliant door swing: the base turns about the hinge while the arm yields to the door.

Each cycle the wrench on the gripper (from the joint efforts) moves the grip radially, up and
down and in yaw (admittance); the base drifts back under the arm so the arm keeps its reach
(re-centering, with the opposite arm motion so the grip stays put); and the grip's path in
odom refines the hinge estimate (circle fit). Only the swing direction is held stiff.
"""

import math
import time

import numpy as np
from frida_constants.manipulation_constants import GRIPPER_REACH
from scipy.spatial.transform import Rotation

from pick_and_place.pipelines.door_swing import (
    SWING_ANGLE,
    SWING_TIMEOUT,
    _rate,
    base_twist,
    swing_sign,
)
from pick_and_place.robot.base import BASE_FRAME

ARM_FRAME = "link_base"
EEF_LINK = "link_eef"
JOINT_LINKS = [f"link{i}" for i in range(1, 7)]
LOOP_PERIOD = 0.05  # s
# +1: efforts read J^T of the wrench on the gripper (the sim); the real xArm sign is to be checked
EFFORT_SIGN = 1.0
BASELINE_SAMPLES = 10
WRENCH_SMOOTHING = 0.3  # EMA weight of the newest wrench
FORCE_DEADBAND = 3.0  # N
TORQUE_DEADBAND = 0.3  # Nm
# Admittance: grip speed per unit of wrench, and its clamps
FORCE_GAIN = 0.002  # (m/s)/N
TORQUE_GAIN = 0.1  # (rad/s)/Nm
MAX_ARM_SPEED = 0.03  # m/s
MAX_ARM_TURN = 0.3  # rad/s
# Re-centering: base speed per metre / radian the grip has drifted in base_link
RECENTER_GAIN = 0.8  # 1/s
MAX_RECENTER_SPEED = 0.03  # m/s
MAX_RECENTER_TURN = 0.1  # rad/s
# Stops: wrench the door may push back with, and how far the grip may leave its spot
FORCE_LIMIT = 40.0  # N
MAX_DRIFT = 0.08  # m
# Hinge fit: arc needed, plausible radius, fit residual, pull toward the guess (point vs guess spread)
FIT_ARC = 15.0  # deg
FIT_RADIUS = (0.5, 1.0)  # m
FIT_RESIDUAL = 0.01  # m
FIT_PRIOR_WEIGHT = 0.06
FIT_POINT_SPACING = 0.01  # m between recorded grip points
ARM_COMMAND_DURATION = 0.3  # s the arm keeps a twist before its watchdog stops it
LOG_PERIOD = 1.0  # s


def jacobian(origins, axes, point) -> np.ndarray:
    """6x6 geometric Jacobian at ``point`` from the joint origins and axes (one frame)."""
    columns = [
        np.concatenate([np.cross(axis, point - origin), axis])
        for origin, axis in zip(origins, axes)
    ]
    return np.array(columns).T


def wrench_from_efforts(jac: np.ndarray, efforts) -> np.ndarray:
    """Wrench (force, torque) at the Jacobian point that best explains ``efforts`` = J^T w."""
    return np.linalg.lstsq(jac.T, np.asarray(efforts, dtype=float), rcond=1e-3)[0]


def deadband(values, width: float) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    return np.sign(values) * np.maximum(np.abs(values) - width, 0.0)


def _clamp_norm(vector, limit: float) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    return vector * (limit / norm) if norm > limit else vector


def radial(grip_xy, hinge_xy) -> np.ndarray:
    """Unit vector from the hinge to the grip in the plane."""
    offset = np.asarray(grip_xy, dtype=float) - np.asarray(hinge_xy, dtype=float)
    return offset / max(float(np.linalg.norm(offset)), 1e-6)


def admittance(force, torque_z: float, radial_xy) -> tuple:
    """Grip velocity (xyz) and yaw rate that yield to ``force`` radially and vertically and to ``torque_z``."""
    out = np.append(np.asarray(radial_xy, dtype=float), 0.0)
    force = deadband(force, FORCE_DEADBAND)
    velocity = FORCE_GAIN * (force.dot(out) * out + force[2] * np.array([0, 0, 1.0]))
    turn = TORQUE_GAIN * float(deadband([torque_z], TORQUE_DEADBAND)[0])
    return _clamp_norm(velocity, MAX_ARM_SPEED), float(
        np.clip(turn, -MAX_ARM_TURN, MAX_ARM_TURN)
    )


def recenter(drift_xy, drift_yaw: float) -> np.ndarray:
    """Base correction (vx, vy, wz) in base_link toward a grip that drifted by ``drift_xy``/``drift_yaw``."""
    linear = _clamp_norm(RECENTER_GAIN * np.asarray(drift_xy), MAX_RECENTER_SPEED)
    turn = float(
        np.clip(RECENTER_GAIN * drift_yaw, -MAX_RECENTER_TURN, MAX_RECENTER_TURN)
    )
    return np.array([linear[0], linear[1], turn])


def base_point_velocity(twist, point_xy) -> np.ndarray:
    """Velocity (xy, base_link) of a point carried by the base moving with ``twist``."""
    vx, vy, wz = twist
    return np.array([vx - wz * point_xy[1], vy + wz * point_xy[0]])


def fit_circle(points, prior=None) -> tuple:
    """Least-squares circle through xy ``points``, its centre pulled toward ``prior``: (centre, radius, rms residual)."""
    points = np.asarray(points, dtype=float)
    a = np.column_stack([2 * points[:, 0], 2 * points[:, 1], np.ones(len(points))])
    b = (points**2).sum(axis=1)
    if prior is not None:
        # A short arc fixes the centre poorly along the radius; the guess holds it there
        a = np.vstack([a, FIT_PRIOR_WEIGHT * np.array([[1.0, 0, 0], [0, 1.0, 0]])])
        b = np.concatenate([b, FIT_PRIOR_WEIGHT * np.asarray(prior, dtype=float)])
    cx, cy, c = np.linalg.lstsq(a, b, rcond=None)[0]
    centre = np.array([cx, cy])
    radius = math.sqrt(max(c + cx**2 + cy**2, 0.0))
    residual = float(
        np.sqrt(np.mean((np.linalg.norm(points - centre, axis=1) - radius) ** 2))
    )
    return centre, radius, residual


def arc_degrees(points, centre) -> float:
    """Angle the xy ``points`` span about ``centre``, from the first to the last."""
    first, last = (np.asarray(p) - centre for p in (points[0], points[-1]))
    return abs(math.degrees(math.atan2(np.cross(first, last), first.dot(last))))


def accept_fit(points, centre, radius: float, residual: float) -> bool:
    return bool(
        FIT_RADIUS[0] <= radius <= FIT_RADIUS[1]
        and residual <= FIT_RESIDUAL
        and arc_degrees(points, centre) >= FIT_ARC
    )


def door_angle(grip_xy, start_xy, hinge_xy, sign: float) -> float:
    """Degrees the grip has turned about the hinge since ``start_xy``, positive opening."""
    first = np.asarray(start_xy) - hinge_xy
    now = np.asarray(grip_xy) - hinge_xy
    return math.degrees(sign * math.atan2(np.cross(first, now), first.dot(now)))


def _in_frame(arm, frame: str, child: str):
    """(position, rotation) of ``child`` in ``frame``, or None."""
    pose = arm.link_pose(child, frame)
    if pose is None:
        return None
    p, q = pose.pose.position, pose.pose.orientation
    return np.array([p.x, p.y, p.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])


def grip_state(arm):
    """Grip (fingertip) and eef positions, eef rotation and the Jacobian at the grip, all in link_base; None without TF."""
    frames = [_in_frame(arm, ARM_FRAME, link) for link in JOINT_LINKS + [EEF_LINK]]
    if any(frame is None for frame in frames):
        return None
    eef, rotation = frames[-1]
    grip = eef + rotation.apply([0.0, 0.0, GRIPPER_REACH])
    jac = jacobian(
        [f[0] for f in frames[:-1]],
        [f[1].apply([0.0, 0.0, 1.0]) for f in frames[:-1]],
        grip,
    )
    return grip, eef, rotation, jac


def _efforts(arm):
    state = arm.joint_state
    if state is None or len(state.effort) < 6:
        return None
    return EFFORT_SIGN * np.array(state.effort[:6], dtype=float)


def _yaw(rotation: Rotation) -> float:
    return float(rotation.as_euler("ZYX")[0])


def compliant_swing(arm, base, hinge, side: str = "right", angle: float = SWING_ANGLE):
    """Turn the base ``angle`` deg about ``hinge`` (odom PointStamped) with the arm compliant; returns degrees the grip turned."""
    sign = swing_sign(side)
    samples = []
    for _ in range(BASELINE_SAMPLES):
        efforts = _efforts(arm)
        if efforts is not None:
            samples.append(efforts)
        time.sleep(0.02)
    arm_in_base = _in_frame(arm, BASE_FRAME, ARM_FRAME)
    start = grip_state(arm)
    start_pose = base.pose()
    if not samples or arm_in_base is None or start is None or start_pose is None:
        arm.logger.error("Door swing: no joint efforts, TF or odometry")
        return 0.0
    baseline = np.median(samples, axis=0)
    arm_offset, arm_rotation = arm_in_base
    to_base = arm_rotation
    to_arm = arm_rotation.inv()
    nominal = (to_base.apply(start[0]) + arm_offset)[:2]
    nominal_yaw = _yaw(to_base * start[2])
    guess = np.array([hinge.point.x, hinge.point.y])
    h = guess.copy()
    path = []
    wrench = np.zeros(6)
    turned = 0.0
    began = last_log = time.time()
    why = "timed out"
    arm.logger.info(
        f"Door swing (compliant): {angle:.0f} deg about the hinge at {np.round(h, 3).tolist()}"
    )
    try:
        with arm.cartesian_velocity_mode("swing_door"):
            while True:
                elapsed = time.time() - began
                arm.check_abort()
                pose = base.pose()
                state = grip_state(arm)
                efforts = _efforts(arm)
                if pose is None or state is None or efforts is None:
                    why = "lost odometry, TF or efforts"
                    break
                grip, eef, rotation, jac = state
                c, s = math.cos(pose[2]), math.sin(pose[2])
                grip_base = (to_base.apply(grip) + arm_offset)[:2]
                grip_odom = pose[:2] + np.array(
                    [
                        c * grip_base[0] - s * grip_base[1],
                        s * grip_base[0] + c * grip_base[1],
                    ]
                )
                if (
                    not path
                    or np.linalg.norm(grip_odom - path[-1]) >= FIT_POINT_SPACING
                ):
                    path.append(grip_odom)
                if len(path) >= 5:
                    centre, radius, residual = fit_circle(path, guess)
                    if accept_fit(path, centre, radius, residual):
                        h = centre
                turned = door_angle(grip_odom, path[0], h, sign)
                if turned >= angle:
                    why = "reached the angle"
                    break
                if elapsed > SWING_TIMEOUT:
                    break
                # Wrench on the grip, in base_link
                measured = wrench_from_efforts(jac, efforts - baseline)
                wrench = (
                    WRENCH_SMOOTHING
                    * np.concatenate(
                        [to_base.apply(measured[:3]), to_base.apply(measured[3:])]
                    )
                    + (1.0 - WRENCH_SMOOTHING) * wrench
                )
                if np.linalg.norm(wrench[:3]) > FORCE_LIMIT:
                    why = f"door pushing back {np.linalg.norm(wrench[:3]):.0f} N"
                    break
                drift = grip_base - nominal
                drift_yaw = math.remainder(
                    _yaw(to_base * rotation) - nominal_yaw, 2 * math.pi
                )
                if np.linalg.norm(drift) > MAX_DRIFT:
                    why = f"grip {np.linalg.norm(drift) * 100:.0f} cm off its spot"
                    break
                # Base: rigid turn about the hinge plus the correction under the arm
                offset = h - pose[:2]
                hinge_local = np.array(
                    [c * offset[0] + s * offset[1], -s * offset[0] + c * offset[1]]
                )
                correction = recenter(drift, drift_yaw)
                twist = np.array(
                    base_twist(hinge_local, sign * _rate(elapsed, turned, angle))
                )
                base.send(*(twist + correction))
                # Arm: yield to the door, minus what the correction carries the grip
                radial_base = radial(grip_odom, h)
                radial_base = np.array(
                    [
                        c * radial_base[0] + s * radial_base[1],
                        -s * radial_base[0] + c * radial_base[1],
                    ]
                )
                velocity, turn = admittance(wrench[:3], wrench[5], radial_base)
                velocity[:2] -= base_point_velocity(correction, grip_base)
                turn -= correction[2]
                # The arm moves its eef; the grip is GRIPPER_REACH beyond it
                angular = to_arm.apply([0.0, 0.0, turn])
                linear = to_arm.apply(velocity) - np.cross(angular, grip - eef)
                arm.send_cartesian_velocity(linear, angular, ARM_COMMAND_DURATION)
                if time.time() - last_log > LOG_PERIOD:
                    last_log = time.time()
                    arm.logger.info(
                        f"Door swing: {turned:.0f} deg, hinge {np.round(h, 3).tolist()}, force {np.round(wrench[:3], 1).tolist()} N, drift {drift[0] * 1000:.0f},{drift[1] * 1000:.0f} mm {math.degrees(drift_yaw):.0f} deg"
                    )
                time.sleep(LOOP_PERIOD)
    finally:
        base.stop()
    arm.logger.info(f"Door swing: turned {turned:.0f} deg ({why})")
    return turned


__all__ = [
    "accept_fit",
    "admittance",
    "arc_degrees",
    "base_point_velocity",
    "compliant_swing",
    "deadband",
    "door_angle",
    "fit_circle",
    "jacobian",
    "radial",
    "recenter",
    "wrench_from_efforts",
]
