"""Compliant door swing: the base turns about the hinge while the arm yields to the door.

Each cycle the base is held on the rigid turn about the hinge from odometry (its distance
to the hinge and its heading toward it, as at the start), which takes out wheel slip; the force
on the gripper (from the joint efforts) moves the grip radially and up and down (admittance);
and the base drifts back under the arm so the arm keeps its reach (re-centering, with the
opposite arm motion so the grip stays put). Only the swing direction is held stiff.
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
# Admittance: grip speed per unit of wrench, and its clamps
FORCE_GAIN = 0.002  # (m/s)/N
MAX_ARM_SPEED = 0.03  # m/s
# Re-centering: base speed per metre / radian the grip has drifted in base_link
RECENTER_GAIN = 0.8  # 1/s
MAX_RECENTER_SPEED = 0.03  # m/s
# Rigid-turn tracking: base speed per metre off its hinge radius and per radian off its heading
TRACK_GAIN = 1.0  # 1/s
TRACK_INTEGRAL_GAIN = 0.5  # 1/s^2
MAX_TRACK_SPEED = 0.04  # m/s
MAX_TRACK_TURN = 0.15  # rad/s
# Stops: wrench the door may push back with, and how far the grip may leave its spot
FORCE_LIMIT = 40.0  # N
MAX_DRIFT = 0.08  # m
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


def admittance(force, radial_xy) -> np.ndarray:
    """Grip velocity (xyz) that yields to ``force`` radially and vertically."""
    out = np.append(np.asarray(radial_xy, dtype=float), 0.0)
    force = deadband(force, FORCE_DEADBAND)
    velocity = FORCE_GAIN * (force.dot(out) * out + force[2] * np.array([0, 0, 1.0]))
    return _clamp_norm(velocity, MAX_ARM_SPEED)


def recenter(drift_xy) -> np.ndarray:
    """Base correction (vx, vy, wz) in base_link toward a grip that drifted by ``drift_xy``."""
    linear = _clamp_norm(RECENTER_GAIN * np.asarray(drift_xy), MAX_RECENTER_SPEED)
    return np.array([linear[0], linear[1], 0.0])


class RigidTracker:
    """Holds the base on a rigid turn about the hinge from odometry: its radius and its heading relative to the hinge."""

    def __init__(self, pose, hinge_xy):
        self.reset(pose, hinge_xy)

    def reset(self, pose, hinge_xy) -> None:
        out = np.asarray(pose[:2]) - hinge_xy
        self.radius = float(np.linalg.norm(out))
        self.heading = float(pose[2] - math.atan2(out[1], out[0]))
        self.integral = np.zeros(2)

    def shift(self, radius: float) -> None:
        """Move the reference radius by what the base was moved on purpose."""
        self.radius += radius

    def correction(self, pose, hinge_xy, dt: float = LOOP_PERIOD) -> np.ndarray:
        """Base correction (vx, vy, wz) in base_link back onto the turn."""
        out = np.asarray(pose[:2]) - hinge_xy
        radius = max(float(np.linalg.norm(out)), 1e-6)
        error = np.array(
            [
                self.radius - radius,
                math.remainder(
                    self.heading + math.atan2(out[1], out[0]) - pose[2], 2 * math.pi
                ),
            ]
        )
        # Integral action takes out a steady slip or heading bias
        self.integral = np.clip(
            self.integral + TRACK_INTEGRAL_GAIN * dt * error,
            [-MAX_TRACK_SPEED, -MAX_TRACK_TURN],
            [MAX_TRACK_SPEED, MAX_TRACK_TURN],
        )
        push, turn = TRACK_GAIN * error + self.integral
        push = float(np.clip(push, -MAX_TRACK_SPEED, MAX_TRACK_SPEED))
        c, s = math.cos(pose[2]), math.sin(pose[2])
        unit = out / radius
        return np.array(
            [
                push * (c * unit[0] + s * unit[1]),
                push * (-s * unit[0] + c * unit[1]),
                float(np.clip(turn, -MAX_TRACK_TURN, MAX_TRACK_TURN)),
            ]
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
    """Grip (fingertip) position and the Jacobian there, in link_base; None without TF."""
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
    return grip, jac


def _efforts(arm):
    state = arm.joint_state
    if state is None or len(state.effort) < 6:
        return None
    return EFFORT_SIGN * np.array(state.effort[:6], dtype=float)


def effort_baseline(arm):
    """Median joint efforts over a short still window, or None."""
    samples = []
    for _ in range(BASELINE_SAMPLES):
        efforts = _efforts(arm)
        if efforts is not None:
            samples.append(efforts)
        time.sleep(0.02)
    return np.median(samples, axis=0) if samples else None


def compliant_swing(
    arm, base, hinge, side: str = "right", angle: float = SWING_ANGLE, baseline=None
):
    """Turn the base ``angle`` deg about ``hinge`` (odom PointStamped) with the arm compliant; returns degrees the grip turned.

    ``baseline``: efforts before the door loaded the grip (just after grasping), else sampled now.
    """
    sign = swing_sign(side)
    if baseline is None:
        baseline = effort_baseline(arm)
    arm_in_base = _in_frame(arm, BASE_FRAME, ARM_FRAME)
    start = grip_state(arm)
    start_pose = base.pose()
    if baseline is None or arm_in_base is None or start is None or start_pose is None:
        arm.logger.error("Door swing: no joint efforts, TF or odometry")
        return 0.0
    arm_offset, arm_rotation = arm_in_base
    to_base = arm_rotation
    to_arm = arm_rotation.inv()
    nominal = (to_base.apply(start[0]) + arm_offset)[:2]
    h = np.array([hinge.point.x, hinge.point.y])
    tracker = RigidTracker(start_pose, h)
    start_grip = None
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
                grip, jac = state
                c, s = math.cos(pose[2]), math.sin(pose[2])
                grip_base = (to_base.apply(grip) + arm_offset)[:2]
                grip_odom = pose[:2] + np.array(
                    [
                        c * grip_base[0] - s * grip_base[1],
                        s * grip_base[0] + c * grip_base[1],
                    ]
                )
                if start_grip is None:
                    start_grip = grip_odom
                turned = door_angle(grip_odom, start_grip, h, sign)
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
                if np.linalg.norm(drift) > MAX_DRIFT:
                    why = f"grip {np.linalg.norm(drift) * 100:.0f} cm off its spot"
                    break
                # Base: rigid turn about the hinge plus the correction under the arm
                offset = h - pose[:2]
                hinge_local = np.array(
                    [c * offset[0] + s * offset[1], -s * offset[0] + c * offset[1]]
                )
                correction = recenter(drift)
                # What the re-centering moves on purpose becomes the new rigid reference
                outward = radial(pose[:2], h)
                moved = np.array(
                    [
                        c * correction[0] - s * correction[1],
                        s * correction[0] + c * correction[1],
                    ]
                )
                tracker.shift(LOOP_PERIOD * moved.dot(outward))
                twist = np.array(
                    base_twist(hinge_local, sign * _rate(elapsed, turned, angle))
                )
                base.send(*(twist + correction + tracker.correction(pose, h)))
                # Arm: yield to the door, minus what the correction carries the grip
                radial_base = radial(grip_odom, h)
                radial_base = np.array(
                    [
                        c * radial_base[0] + s * radial_base[1],
                        -s * radial_base[0] + c * radial_base[1],
                    ]
                )
                velocity = admittance(wrench[:3], radial_base)
                velocity[:2] -= correction[:2]
                arm.send_cartesian_velocity(
                    to_arm.apply(velocity), duration=ARM_COMMAND_DURATION
                )
                if time.time() - last_log > LOG_PERIOD:
                    last_log = time.time()
                    arm.logger.info(
                        f"Door swing: {turned:.0f} deg, hinge {np.round(h, 3).tolist()}, force {np.round(wrench[:3], 1).tolist()} N, drift {drift[0] * 1000:.0f},{drift[1] * 1000:.0f} mm"
                    )
                time.sleep(LOOP_PERIOD)
    finally:
        base.stop()
    arm.logger.info(f"Door swing: turned {turned:.0f} deg ({why})")
    return turned


__all__ = [
    "admittance",
    "compliant_swing",
    "deadband",
    "door_angle",
    "effort_baseline",
    "jacobian",
    "radial",
    "recenter",
    "RigidTracker",
    "wrench_from_efforts",
]
