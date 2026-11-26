"""Swing an unlatched pull door open: the base turns about the hinge while the arm holds the lever still.

Turning the whole robot rigidly about the hinge axis moves the gripper exactly as the
door moves its handle, so the arm never has to follow the arc itself.
"""

import math
import time

import numpy as np
from geometry_msgs.msg import PointStamped

from pick_and_place.robot.base import ODOM_FRAME

DOOR_WIDTH = 0.80  # m, leaf width
LEVER_PIVOT_INSET = 0.07  # m from the latch edge to the lever pivot
# The lever tip points at the hinge; "right" as seen from the robot
HINGE_SIDE = "right"
SWING_ANGLE = 75.0  # deg the base turns
SWING_RATE = 0.12  # rad/s
RAMP_TIME = 1.0  # s to reach SWING_RATE
SLOW_DOWN = 10.0  # deg before the end the rate ramps down
MIN_RATE = 0.03  # rad/s at the end of the ramp
LOOP_PERIOD = 0.05  # s
# N(m) any arm joint may rise above its start effort before the swing stops
EFFORT_LIMIT = 15.0
EFFORT_SAMPLES = 10
SWING_TIMEOUT = 40.0  # s
# Where the base lines up for the grasp: handle centre in base_link, door square on
HANDLE_AHEAD = 0.69  # m
HANDLE_SIDE = 0.03  # m, + to the robot's left
ALIGN_TOLERANCE = 0.015  # m
ALIGN_HEADING_TOLERANCE = math.radians(2.0)


def hinge_point(handle, side: str = HINGE_SIDE) -> np.ndarray:
    """Hinge axis (x, y) in the handle's frame, from the lever on the door face."""
    c, n = handle.center.point, handle.door_normal
    centre = np.array([c.x, c.y])
    normal = np.array([n.x, n.y])
    normal /= np.linalg.norm(normal)
    # z x normal: the robot's right while it faces the door
    right = np.array([-normal[1], normal[0]])
    toward = right if side == "right" else -right
    reach = DOOR_WIDTH - LEVER_PIVOT_INSET - handle.length / 2.0
    return centre - normal * handle.standoff + toward * reach


def swing_sign(side: str = HINGE_SIDE) -> float:
    """Yaw direction that pulls the door toward the robot (counter-clockwise for a right hinge)."""
    return 1.0 if side == "right" else -1.0


def base_twist(hinge_xy: np.ndarray, rate: float) -> tuple:
    """(vx, vy, wz) in the base frame turning the base at ``rate`` about ``hinge_xy`` (base frame)."""
    # v = w z x (0 - h)
    return rate * hinge_xy[1], -rate * hinge_xy[0], rate


def alignment(handle) -> np.ndarray:
    """(dx, dy, dyaw) in the base frame that puts the handle at the grasp spot with the door square on."""
    c, n = handle.center.point, handle.door_normal
    # Turning by the door yaw points the door normal straight back at the robot
    yaw = math.atan2(-n.y, -n.x)
    cos, sin = math.cos(yaw), math.sin(yaw)
    centre = np.array([cos * c.x + sin * c.y, -sin * c.x + cos * c.y])
    shift = centre - np.array([HANDLE_AHEAD, HANDLE_SIDE])
    return np.array(
        [cos * shift[0] - sin * shift[1], sin * shift[0] + cos * shift[1], yaw]
    )


def alignment_needed(handle) -> bool:
    """Whether the handle is off the grasp spot by more than the alignment tolerances."""
    move = alignment(handle)
    return bool(
        np.linalg.norm(move[:2]) >= ALIGN_TOLERANCE
        or abs(move[2]) >= ALIGN_HEADING_TOLERANCE
    )


def align(arm, base, handle) -> bool:
    """Drive the base so the handle sits at the grasp spot; False when it could not."""
    move = alignment(handle)
    pose = base.pose()
    if pose is None:
        return False
    cos, sin = math.cos(pose[2]), math.sin(pose[2])
    target = np.array(
        [
            pose[0] + cos * move[0] - sin * move[1],
            pose[1] + sin * move[0] + cos * move[1],
            pose[2] + move[2],
        ]
    )
    arm.logger.info(
        f"Door: lining the base up by ({move[0] * 1000:.0f}, {move[1] * 1000:.0f}) mm, {math.degrees(move[2]):.1f} deg"
    )
    return base.move_to(
        target,
        tolerance=ALIGN_TOLERANCE / 2,
        heading_tolerance=ALIGN_HEADING_TOLERANCE / 2,
    )


def odom_hinge(arm, handle, side: str = HINGE_SIDE):
    """The hinge as an odom PointStamped, or None without TF."""
    x, y = hinge_point(handle, side)
    point = PointStamped()
    point.header.frame_id = handle.center.header.frame_id
    point.point.x, point.point.y, point.point.z = float(x), float(y), 0.0
    return arm.to_frame(point, ODOM_FRAME)


def _efforts(arm) -> np.ndarray | None:
    state = arm.joint_state
    if state is None or len(state.effort) < 6:
        return None
    return np.array(state.effort[:6], dtype=float)


def _rate(elapsed: float, turned: float, target: float) -> float:
    """Ramped turn rate (rad/s) for ``turned`` of ``target`` degrees."""
    up = SWING_RATE * min(1.0, elapsed / RAMP_TIME)
    left = target - turned
    down = SWING_RATE * max(0.0, min(1.0, left / SLOW_DOWN))
    return max(MIN_RATE, min(up, down))


def swing_open(arm, base, hinge, side: str = HINGE_SIDE, angle: float = SWING_ANGLE):
    """Turn the base ``angle`` deg about ``hinge`` (odom PointStamped); returns degrees turned."""
    start = base.pose()
    samples = []
    for _ in range(EFFORT_SAMPLES):
        efforts = _efforts(arm)
        if efforts is not None:
            samples.append(efforts)
        time.sleep(0.02)
    if start is None or not samples:
        arm.logger.error("Door swing: no odometry or joint efforts")
        return 0.0
    baseline = np.median(samples, axis=0)
    sign = swing_sign(side)
    h = np.array([hinge.point.x, hinge.point.y])
    radius = float(np.linalg.norm(h - start[:2]))
    arm.logger.info(
        f"Door swing: turning {angle:.0f} deg about the hinge {radius:.2f} m away"
    )
    turned = 0.0
    began = time.time()
    try:
        while True:
            elapsed = time.time() - began
            arm.check_abort()
            pose = base.pose()
            if pose is None:
                arm.logger.error("Door swing: lost odometry")
                break
            turned = math.degrees(
                sign * math.remainder(pose[2] - start[2], 2 * math.pi)
            )
            if turned >= angle:
                break
            if elapsed > SWING_TIMEOUT:
                arm.logger.warn(f"Door swing: timed out at {turned:.0f} deg")
                break
            efforts = _efforts(arm)
            if efforts is not None:
                rise = float(np.max(np.abs(efforts - baseline)))
                if rise > EFFORT_LIMIT:
                    arm.logger.warn(
                        f"Door swing: arm effort up {rise:.1f} at {turned:.0f} deg, stopping"
                    )
                    break
            yaw = pose[2]
            offset = h - pose[:2]
            # The hinge in the base frame
            local = np.array(
                [
                    math.cos(yaw) * offset[0] + math.sin(yaw) * offset[1],
                    -math.sin(yaw) * offset[0] + math.cos(yaw) * offset[1],
                ]
            )
            base.send(*base_twist(local, sign * _rate(elapsed, turned, angle)))
            time.sleep(LOOP_PERIOD)
    finally:
        base.stop()
    arm.logger.info(f"Door swing: turned {turned:.0f} deg")
    return turned


__all__ = [
    "align",
    "alignment",
    "alignment_needed",
    "base_twist",
    "hinge_point",
    "odom_hinge",
    "swing_open",
    "swing_sign",
]
