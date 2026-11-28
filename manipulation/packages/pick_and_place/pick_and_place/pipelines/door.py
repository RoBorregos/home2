"""OPEN_DOOR: find the handle, grasp it, press the lever, push, else pull and swing the door open with the base."""

import copy
import math
import time
from dataclasses import replace

import numpy as np
from frida_constants.manipulation_constants import (
    DOOR_HANDLE_LABEL,
    DOOR_HANDLE_STARE_POSE,
    FAST_VELOCITY,
    GRASP_LINK_FRAME,
    GRIPPER_MAX_APERTURE,
    GRIPPER_REACH,
    SHELF_SURVEY_SETTLE,
)
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from pick_and_place.pipelines import door_compliance, door_swing
from pick_and_place.pipelines.profiles import ForceGuardProfile
from pick_and_place.robot.geometry import offset_along_approach, set_quaternion

PUSH = np.array([0.0, -1.0, 0.0])
PULL = np.array([0.0, 1.0, 0.0])
APPROACH = PUSH
DOWN = np.array([0.0, 0.0, -1.0])
# deg down, in order: a pitch puts the upper finger deeper.
PITCHES = (0.0, 10.0, 20.0)
# m the deepest fingertip stays in front of the measured door.
FINGER_CLEARANCE = 0.025
# m from the grasp axis to an open finger's outer edge.
FINGER_SPAN = GRIPPER_MAX_APERTURE / 2 + 0.015
# deg joint 1 may turn to the pre-grasp (more swings round the robot, through unseen space).
MAX_BASE_TURN = 90.0
MAX_PREGRASP_SWING = 130.0  # deg any joint may move to reach the pre-grasp
GRASP_VELOCITY = 0.3  # the joint move to the pre-grasp, next to the door
GRASP_PAST = 0.03  # m the fingertips go past the handle's front
PRE_GRASP = 0.08  # m back from the grasp, closed in on by the guarded approach
# m fixed drop pressing the lever (an effort stop ends too early).
LEVER_DROP = 0.06
PRESS_SPEED = 12.0  # mm/s
# deg rolled during the press: + for a pivot left of the grasp, - right.
LEVER_ROLL = 40.0
LEVER_VELOCITY = 0.1
# deg joint 5 turns to press the lever.
LEVER_TURN = 25.0
OPEN_DISTANCE = 0.15  # m of straight pull the grasp check covers
# m the handle may leave the door's arc in a straight pull: pull = R sin(acos(1 - tol / R)).
HINGE_TOLERANCE = 0.02
FALLBACK_PULL = 0.10  # m pulled when the hinge distance is unknown
UNLATCH_DISTANCE = 0.02  # m driven with the lever down, then the lever is let back up
# s for a fresh vision door_handle point (the gallery runs at ~1.2 Hz).
GALLERY_WAIT = 3.0
# Joint 5 must move the fingertips mostly up/down to press a lever, not sideways.
MIN_VERTICAL_SWING = 0.7
# rad/s any joint may need along a drive, as the plate's wrist singularity skip (xArm C24).
MAX_DRIVE_JOINT_SPEED = 0.5
# The plate's guard for sideways drives (free air jumps ~1.4 N on j3/j4).
OPEN_GUARD = ForceGuardProfile(
    descent_speed=12.0,  # mm/s
    timeout=16.0,  # s, covers OPEN_DISTANCE at 12 mm/s
    grace_period=0.3,
    contact_settle=1.0,  # s, skip the motion-onset spike, as the plate
    min_contact_descent=0.005,
    # As the plate: J1/J6 see little, J2's gravity drift false-trips.
    ignore_joints=(0, 1, 5),
    jump_window=0.3,
    jump_trip=3.0,  # N
    jump_sustain=5,
    hard_ceiling=6.0,  # N, push/pull stop before ramming the door (approach keeps 15)
    max_descent=OPEN_DISTANCE,
)
PULL_SPEED = (
    8.0  # mm/s, the unguarded pull once the push showed it does not open that way
)
# Closing in on the handle, stopping at the door; contact only counts in the last 3.5 cm.
APPROACH_GUARD = replace(
    OPEN_GUARD,
    max_descent=PRE_GRASP,
    min_contact_descent=PRE_GRASP - 0.035,
    hard_ceiling=15.0,
)


# m the gripper backs off the lever after letting go of a swung door
RETREAT_DISTANCE = 0.08
# "compliant": the arm yields to the door while the base turns; "rigid": the arm holds still
SWING_MODE = "compliant"
SWING_MODES = ("compliant", "rigid")


def execute(
    arm, perception, base=None, door_width=door_swing.DOOR_WIDTH, swing_mode=SWING_MODE
) -> bool:
    """Go to the door handle stare pose, find the handle and open the door with it (swung with ``base``)."""
    arm.set_context("open_door")
    with arm.phase("door_handle_stare"):
        if not arm.move_to_named_position(
            DOOR_HANDLE_STARE_POSE, velocity=FAST_VELOCITY
        ):
            arm.logger.error(f"Could not reach {DOOR_HANDLE_STARE_POSE}")
            return False
    # The depth and TF catch up, as after a shelf survey pose.
    time.sleep(SHELF_SURVEY_SETTLE)
    with arm.phase("detect_door_handle"):
        seed = vision_point(perception)
        handle = perception.detect_door_handle(seed=seed)  # logs its own failure
        if handle is None and seed is not None:
            # A big gallery box puts its centre point on the wall.
            arm.logger.warn(
                "Nothing lever-like at the vision point: segmenting the whole view"
            )
            handle = perception.detect_door_handle()
    if handle is None:
        return False
    arm.logger.info(f"Door handle {describe(handle)}")
    if base is not None:
        handle = line_up(arm, perception, base, handle)
        if handle is None:
            return False
    return pick_handle(arm, handle, base, door_width, swing_mode)


def line_up(arm, perception, base, handle):
    """Move the base to the grasp spot in front of the handle and measure it again; None on failure."""
    with arm.phase("line_up_base"):
        if door_swing.alignment_needed(handle) and not door_swing.align(
            arm, base, handle
        ):
            arm.logger.error("Door: could not line the base up with the handle")
            return None
    if not door_swing.alignment_needed(handle):
        return handle
    time.sleep(SHELF_SURVEY_SETTLE)
    with arm.phase("detect_door_handle"):
        again = perception.detect_door_handle()
    if again is not None:
        arm.logger.info(f"Door handle after lining up {describe(again)}")
    return again


def vision_point(perception):
    """The vision model's handle point to segment at, or None to segment the whole view."""
    point = perception.gallery_point(DOOR_HANDLE_LABEL, wait=GALLERY_WAIT)
    if point is None:
        perception.logger.info(
            f"No vision {DOOR_HANDLE_LABEL} in {GALLERY_WAIT:.0f} s: segmenting the whole view"
        )
        return None
    perception.logger.info(
        f"Vision {DOOR_HANDLE_LABEL} at ({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f}) {point.header.frame_id}"
    )
    return point


def pick_handle(
    arm, handle, base=None, door_width=door_swing.DOOR_WIDTH, swing_mode=SWING_MODE
) -> bool:
    """Grasp straight on (-y), press the lever, push (-y), else pull (+y) and swing with ``base``, let go."""
    poses = grasp_poses(arm, handle)
    if not poses:
        arm.logger.error("Door grasp: no TF from the handle to link_base")
        return False
    with arm.phase("choose_grasp"):
        choice = choose_grasp(arm, poses)
    if choice is None:
        arm.logger.error("Door grasp: no candidate passed the checks")
        return False
    label, joints = choice
    with arm.phase("open_gripper"):
        arm.open_gripper(settle_s=0.0)
    with arm.phase("approach_pre_grasp"):
        # To the checked IK solution; no octomap clear, so MoveIt still avoids the door.
        if not arm.move_joints(joints, velocity=GRASP_VELOCITY):
            arm.logger.error(f"Door pre-grasp ({label}): the joint move failed")
            return False
        arm.logger.info(f"Door pre-grasp reached ({label})")
    with arm.phase("grasp_handle"):
        arm.check_abort()
        # Stops where the fingertips touch the door, as the plate's contact drive.
        result = arm.force_guarded_descent(APPROACH_GUARD, APPROACH)
        if result.descended_m <= 0.0:
            arm.logger.error("Door handle approach could not start")
            return let_go(arm, False)
        arm.logger.info(
            f"Door handle approach: {'touched the door' if result.contact else 'no contact'} after {result.descended_m * 1000:.0f} mm"
        )
        arm.close_gripper()
    with arm.phase("press_lever"):
        unpress = press_lever(arm)
        if unpress is None:
            return let_go(arm, False)
    with arm.phase("push_door"):
        arm.check_abort()
        opened, _, latched = guarded_open(arm, PUSH, "push", unpress)
    # Pull only when the push never unlatched (the lever is still held down); no force guard.
    if not opened and latched:
        with arm.phase("pull_door"):
            arm.check_abort()
            hinge = float(getattr(handle, "hinge_distance", 0.0))
            pull = pull_distance(hinge)
            arm.logger.info(
                f"Door pull: {pull * 1000:.0f} mm straight (hinge {hinge * 1000:.0f} mm away, {HINGE_TOLERANCE * 1000:.0f} mm off the arc)"
            )
            opened = pull_open(arm, unpress, pull)
        if opened and base is not None:
            return swing(arm, base, handle, door_width, swing_mode)
    return let_go(arm, opened)


def swing(
    arm, base, handle, door_width=door_swing.DOOR_WIDTH, swing_mode=SWING_MODE
) -> bool:
    """Turn the base about the hinge holding the lever, then let go and back off it."""
    with arm.phase("swing_door"):
        hinge = door_swing.odom_hinge(arm, handle, width=door_width)
        if hinge is None:
            arm.logger.error("Door swing: no TF from the handle to odom")
            return let_go(arm, False)
        if swing_mode == "compliant":
            turned = door_compliance.compliant_swing(arm, base, hinge)
        else:
            turned = door_swing.swing_open(arm, base, hinge)
    let_go(arm, True)
    with arm.phase("retreat_from_door"):
        if not arm.cartesian_approach(PULL, RETREAT_DISTANCE, PULL_SPEED):
            arm.logger.warn("Door: backing off the lever did not complete")
    opened = turned >= door_swing.SWING_ANGLE - 5.0
    arm.logger.info(
        f"Door {'open' if opened else 'only partly open'}: swung {turned:.0f} deg"
    )
    return opened


def grasp_poses(arm, handle) -> list:
    """(label, link_base pose) straight-on grasps per pitch, camera toward the middle first."""
    centre = arm.to_frame(handle.center, "link_base")
    if centre is None:
        return []
    point = centre.point
    front = np.array([point.x, point.y, point.z])
    # link_base x is the robot's left: a handle on the left puts the camera on the right first.
    ups = [1.0, -1.0] if point.x > 0 else [-1.0, 1.0]
    poses = []
    for pitch in PITCHES:
        down = math.radians(pitch)
        # The deepest fingertip stays FINGER_CLEARANCE off the door.
        deepest = FINGER_SPAN * math.sin(down)
        room = handle.standoff - FINGER_CLEARANCE - deepest
        past = min(GRASP_PAST, room) if handle.standoff > 0 else GRASP_PAST - deepest
        tip = front + APPROACH * past
        axis = np.array([0.0, -math.cos(down), -math.sin(down)])
        for up in ups:
            # The fingers close along the grasp frame's y: about vertical, across the lever.
            closing = up * np.array([0.0, -math.sin(down), math.cos(down)])
            side = np.cross(closing, axis)
            pose = PoseStamped()
            pose.header.frame_id = "link_base"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
                float, tip
            )
            quat = Rotation.from_matrix(
                np.column_stack((side, closing, axis))
            ).as_quat()
            set_quaternion(pose, quat)
            # The ZED sits on the grasp frame's -x.
            camera = "left" if side[0] < 0 else "right"
            label = f"pitch {pitch:.0f} deg, camera {camera}"
            grasp = offset_along_approach(pose, arm.tip_offset("geometric_tip_offset"))
            poses.append((label, grasp))
    return poses


def choose_grasp(arm, poses):
    """(label, pre-grasp joints) of the first candidate passing every check, or None."""
    for label, grasp in poses:
        ik = arm.nearest_ik(_shifted(grasp, -APPROACH * PRE_GRASP))
        if ik is None:
            arm.logger.warn(f"Door grasp ({label}) skipped: pre-grasp unreachable")
            continue
        joints, change, solution = ik
        swing = float(np.abs(change).max())
        if abs(change[0]) > MAX_BASE_TURN or swing > MAX_PREGRASP_SWING:
            arm.logger.warn(
                f"Door grasp ({label}) skipped: joint changes {change.round(0).tolist()} deg"
            )
            continue
        why = drive_blocked(arm, grasp, solution)
        if why:
            arm.logger.warn(f"Door grasp ({label}) skipped: {why}")
            continue
        arm.logger.info(
            f"Door grasp ({label}) chosen: joint changes {change.round(0).tolist()} deg"
        )
        return label, joints
    return None


def drive_blocked(arm, grasp, seed=None) -> str:
    """Why the approach or push/pull drive crosses the wrist singularity, or ""."""
    drives = (
        (
            _shifted(grasp, -APPROACH * PRE_GRASP),
            APPROACH,
            PRE_GRASP,
            APPROACH_GUARD.descent_speed,
        ),
        # The unlatch pull back to the unlatch push, at the pressed lever's height.
        (
            _shifted(grasp, PULL * UNLATCH_DISTANCE + DOWN * LEVER_DROP),
            PUSH,
            2 * UNLATCH_DISTANCE,
            OPEN_GUARD.descent_speed,
        ),
        # The straight pull at the grasp height (about a 0.6-0.9 m hinge).
        (grasp, PULL, OPEN_DISTANCE, PULL_SPEED),
    )
    # Seeded with the pre-grasp solution, so the drives are checked on the branch the arm will be in.
    for start, direction, distance, speed in drives:
        rate = arm.drive_joint_rate(start, direction, distance, seed)
        if rate is None:
            return "no IK along the drive"
        joint_speed = rate * speed / 1000.0
        if joint_speed > MAX_DRIVE_JOINT_SPEED:
            return f"drive crosses the wrist singularity ({joint_speed:.2f} rad/s)"
    return ""


def press_lever(arm):
    """Press with joint 5, else straight down; returns the undo, None on failure."""
    joint5, grasp = arm.link_pose("link5"), arm.link_pose(GRASP_LINK_FRAME)
    if joint5 is None or grasp is None:
        return None
    axis = _rotation(joint5).apply([0.0, 0.0, 1.0])
    tip = _position(grasp) + _rotation(grasp).apply([0.0, 0.0, GRIPPER_REACH])
    # Fingertip motion per radian of +joint5.
    swing = np.cross(axis, tip - _position(joint5))
    radius = float(np.linalg.norm(swing))
    if radius < 1e-3 or abs(swing[2]) < MIN_VERTICAL_SWING * radius:
        # A handle off to the side rolls joint 4, so joint 5 swings sideways.
        arm.logger.info(
            "Joint 5 swings the gripper sideways here: pressing the lever straight down"
        )
        # Roll about +x_base, ending with the drop.
        roll_rate = math.radians(LEVER_ROLL) * PRESS_SPEED / (LEVER_DROP * 1000)
        if not arm.cartesian_approach(
            DOWN, LEVER_DROP, PRESS_SPEED, angular=APPROACH * roll_rate
        ):
            arm.logger.error(
                f"Pressing the lever {LEVER_DROP * 1000:.0f} mm down did not complete"
            )
            return None
        arm.logger.info(
            f"Lever pressed {LEVER_DROP * 1000:.0f} mm, rolled {LEVER_ROLL:.0f} deg"
        )
        return lambda: arm.cartesian_approach(
            -DOWN, LEVER_DROP, PRESS_SPEED, angular=-APPROACH * roll_rate
        )
    turn = -math.copysign(LEVER_TURN, swing[2])
    joints = arm.get_joints(degrees=True)
    joints["joints"]["joint5"] += turn
    arm.logger.info(
        f"Pressing the lever: joint 5 {turn:+.1f} deg, {radius * math.sin(math.radians(LEVER_TURN)) * 1000:.0f} mm down at the fingertips"
    )
    arm.clear_octomap()
    if not arm.move_joints(joints, velocity=LEVER_VELOCITY):
        return None

    def unpress():
        back = arm.get_joints(degrees=True)
        back["joints"]["joint5"] -= turn
        return arm.move_joints(back, velocity=LEVER_VELOCITY)

    return unpress


def guarded_open(arm, direction, name: str, unpress):
    """Unlatch under OPEN_GUARD, then ``unpress``: (opened, m moved, latched)."""
    first = arm.force_guarded_descent(
        replace(OPEN_GUARD, max_descent=UNLATCH_DISTANCE), direction
    )
    moved = first.descended_m
    if first.contact or moved < UNLATCH_DISTANCE:
        why = "too much effort" if first.contact else "stalled"
        arm.logger.warn(
            f"Door {name}: {why} after {moved * 1000:.0f} mm, it does not open this way"
        )
        return False, moved, True
    arm.logger.info(f"Door {name}: unlatched, back up to the grasp height")
    if not unpress():
        arm.logger.warn(
            f"Door {name}: going back up to the grasp height did not complete"
        )
        return False, moved, False
    arm.logger.info(f"Door opened by a {name} of {moved * 1000:.0f} mm")
    return True, moved, False


def pull_open(arm, unpress, distance: float) -> bool:
    """Pull UNLATCH_DISTANCE with the lever down, ``unpress``, pull the rest; cartesian, no force guard."""
    if not arm.cartesian_approach(PULL, UNLATCH_DISTANCE, PULL_SPEED):
        arm.logger.warn("Door pull: the unlatch pull did not complete")
        return False
    arm.logger.info("Door pull: unlatched, back up to the grasp height")
    if not unpress():
        arm.logger.warn("Door pull: going back up to the grasp height did not complete")
        return False
    if not arm.cartesian_approach(PULL, distance - UNLATCH_DISTANCE, PULL_SPEED):
        arm.logger.warn("Door pull: the rest of the pull did not complete")
        return False
    arm.logger.info(f"Door opened by a pull of {distance * 1000:.0f} mm")
    return True


def pull_distance(hinge: float) -> float:
    """Straight pull that keeps the handle within HINGE_TOLERANCE of its arc about the hinge."""
    if hinge <= HINGE_TOLERANCE:
        return FALLBACK_PULL
    return hinge * math.sin(math.acos(1.0 - HINGE_TOLERANCE / hinge))


def let_go(arm, result: bool) -> bool:
    """Open the gripper and stay there; returns ``result``."""
    with arm.phase("release_handle"):
        arm.open_gripper()
    return result


def describe(handle) -> str:
    """One line with where the handle is and how big (a DetectDoorHandle response)."""
    c, n = handle.center.point, handle.door_normal
    yaw = math.degrees(math.atan2(-n.y, -n.x))
    return (
        f"at ({c.x:.3f}, {c.y:.3f}, {c.z:.3f}) {handle.center.header.frame_id} | length {handle.length * 1000:.0f} mm, "
        f"thickness {handle.thickness * 1000:.0f} mm, stand-off {handle.standoff * 1000:.0f} mm | door yaw {yaw:.1f} deg, "
        f"fit {handle.residual * 1000:.1f} mm | frames {handle.frames_found}/{handle.frames_used}"
    )


def _shifted(pose, vector) -> PoseStamped:
    """Copy of ``pose`` moved by ``vector`` (m, in its frame), same orientation."""
    moved = copy.deepcopy(pose)
    p = moved.pose.position
    p.x, p.y, p.z = (float(a + b) for a, b in zip((p.x, p.y, p.z), vector))
    return moved


def _position(pose) -> np.ndarray:
    p = pose.pose.position
    return np.array([p.x, p.y, p.z])


def _rotation(pose) -> Rotation:
    o = pose.pose.orientation
    return Rotation.from_quat([o.x, o.y, o.z, o.w])


__all__ = [
    "choose_grasp",
    "describe",
    "drive_blocked",
    "execute",
    "grasp_poses",
    "guarded_open",
    "pick_handle",
    "press_lever",
    "pull_open",
    "pull_distance",
    "swing",
]
