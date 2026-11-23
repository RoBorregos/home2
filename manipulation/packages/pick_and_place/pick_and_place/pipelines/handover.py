"""The take-from-hand pipeline: grip the handle of a bag a person holds out.

    wrist point -> base_link -> handle point below the wrist -> reach check
    -> open gripper -> pre-grasp at the first usable yaw -> straight approach,
    closing on reach -> wait for the person to let go -> back away and up

The person is first shown how to hold the bag: arm extended towards the robot,
the handle gripped from above with one hand and the bag hanging. The bag's
weight keeps the handle taut and vertical under the hand, so the grasp is fixed
relative to the detected wrist and the handle itself is never detected.

Safety rules, each with a test:

- The target is always below the wrist, never the wrist itself.
- A blocked approach (most likely the hand) opens the gripper, backs out along
  the same line and fails; it never tries another angle towards the person.
- Without a payload the arm does not retreat, which could pull the bag out of a
  hand still holding it; the caller confirms by voice instead.
- An out-of-reach handle or a missing TF fails before the arm moves.

PickAborted and PickHardwareError propagate: the core decides what to do.
"""

from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from perception_3d.geometric_grasps import grasp_frame
from scipy.spatial.transform import Rotation

from pick_and_place.robot.geometry import offset_along_approach, set_quaternion

# Frame the geometry is computed in, and the arm's own base frame. The xArm TCP
# (cartesian_approach) moves in the arm frame, yawed 90 degrees from base_link.
BASE_FRAME = "base_link"
ARM_FRAME = "link_base"

# --- tuning: calibrate on the robot ------------------------------------------
# Grip point below the detected wrist keypoint, on the taut handle (m).
GRASP_BELOW_WRIST = 0.08
# Handle positions the arm can take from: horizontal distance from link_base (m)
# and height above the floor, i.e. z in base_link (m).
REACH_HORIZONTAL = (0.25, 0.65)
REACH_HEIGHT = (0.30, 1.40)
# Pre-grasp stand-off behind the handle along the approach (m); also the length
# of the straight approach that closes on it.
PREGRASP_DISTANCE = 0.12
# Velocity scaling of the planned move to the pre-grasp: slow, a person is near.
PREGRASP_VELOCITY = 0.2
# Straight approach speed onto the handle (mm/s).
APPROACH_SPEED = 40.0
# Approach yaws tried in order, about world Z, from the straight line from the
# arm base to the handle (degrees).
APPROACH_YAWS_DEG = (0, 15, -15, 30, -30)
# Effort rise on these joints that means the arm now carries the bag (Nm).
PAYLOAD_EFFORT_DELTA = 1.5
PAYLOAD_JOINTS = ("joint2", "joint3", "joint5")
# How long the person gets to let go once the gripper closed (s).
RELEASE_TIMEOUT = 8.0
# Retreat with the bag, back towards the robot and up (m, mm/s).
RETREAT_DISTANCE = 0.12
RETREAT_SPEED = 40.0
# -----------------------------------------------------------------------------

# Below this horizontal distance the handle is over the arm base and has no
# horizontal approach (m).
MIN_HORIZONTAL_DISTANCE = 1e-3

WORLD_UP = np.array([0.0, 0.0, 1.0])


@dataclass
class TakeFromHandRequest:
    """Everything the take needs from the caller; values <= 0 mean the default."""

    hand_point: PointStamped
    grasp_below_hand: float = 0.0
    release_timeout: float = 0.0

    def __post_init__(self):
        if self.grasp_below_hand <= 0:
            self.grasp_below_hand = GRASP_BELOW_WRIST
        if self.release_timeout <= 0:
            self.release_timeout = RELEASE_TIMEOUT


@dataclass
class TakeFromHandOutcome:
    """success: the gripper closed on the handle; payload_detected: the person let go."""

    success: bool = False
    payload_detected: bool = False
    message: str = ""


# ======================================================================
# Geometry
# ======================================================================


def handle_point(wrist: np.ndarray, below: float) -> np.ndarray:
    """The grip point on the handle, ``below`` metres under the wrist."""
    if below <= 0:
        raise ValueError("the grip point must be below the wrist")
    return np.asarray(wrist, dtype=float) - below * WORLD_UP


def within_reach(target: np.ndarray, arm_base: np.ndarray) -> bool:
    """True if ``target`` (base_link) lies in the arm's take-from-hand workspace."""
    horizontal = float(
        np.linalg.norm(np.asarray(target)[:2] - np.asarray(arm_base)[:2])
    )
    return (
        REACH_HORIZONTAL[0] <= horizontal <= REACH_HORIZONTAL[1]
        and REACH_HEIGHT[0] <= float(target[2]) <= REACH_HEIGHT[1]
    )


def approach_direction(
    target: np.ndarray, arm_base: np.ndarray, yaw: float
) -> np.ndarray:
    """Horizontal unit approach from the arm base towards ``target``, turned ``yaw`` rad.

    Raises ValueError when the target is straight above the arm base.
    """
    direction = np.asarray(target, dtype=float) - np.asarray(arm_base, dtype=float)
    direction[2] = 0.0
    norm = np.linalg.norm(direction)
    if norm < MIN_HORIZONTAL_DISTANCE:
        raise ValueError("handle straight above the arm base: no horizontal approach")
    return Rotation.from_euler("z", yaw).apply(direction / norm)


def handle_grasp_quat(approach: np.ndarray) -> np.ndarray:
    """Grasp orientation [x, y, z, w] with the fingers closing across the handle.

    The fingers close along the grasp frame's local Y; a horizontal closing axis
    perpendicular to the approach puts the vertical handle between them.
    """
    return grasp_frame(np.asarray(approach, dtype=float), np.cross(WORLD_UP, approach))


def retreat_direction(approach: np.ndarray) -> np.ndarray:
    """Unit direction back along the approach and up, at 45 degrees."""
    back = -np.asarray(approach, dtype=float)
    back[2] = 0.0
    back /= np.linalg.norm(back)
    direction = back + WORLD_UP
    return direction / np.linalg.norm(direction)


def grasp_pose(
    target: np.ndarray, approach: np.ndarray, tip_offset: float
) -> PoseStamped:
    """Grasp-link pose in base_link whose fingertips close on ``target``.

    ``tip_offset`` is negative: the grasp link sits behind the fingertips.
    """
    position = np.asarray(target, dtype=float) + np.asarray(approach) * tip_offset
    pose = PoseStamped()
    pose.header.frame_id = BASE_FRAME
    pose.pose.position.x = float(position[0])
    pose.pose.position.y = float(position[1])
    pose.pose.position.z = float(position[2])
    return set_quaternion(pose, handle_grasp_quat(approach))


# ======================================================================
# Pipeline
# ======================================================================


def execute(arm, request: TakeFromHandRequest) -> TakeFromHandOutcome:
    """Grip the bag's handle below the person's wrist and take it once released."""
    log = arm.logger
    arm.set_context("take_from_hand")

    with arm.phase("locate_handle"):
        located = _locate_handle(arm, request)
    if isinstance(located, TakeFromHandOutcome):
        return located
    target, arm_base, to_arm = located

    arm.check_abort()
    # No settle: the gripper finishes opening during the pre-grasp move.
    with arm.phase("open_gripper"):
        arm.open_gripper(settle_s=0.0)

    approach = _reach_pregrasp(arm, target, arm_base)
    if approach is None:
        return _failed(log, "no reachable pre-grasp for the handle")

    arm.check_abort()
    with arm.phase("approach"):
        reached = arm.cartesian_approach(
            to_arm.apply(approach),
            PREGRASP_DISTANCE,
            APPROACH_SPEED,
            close_on_reach=True,
        )
    if not reached:
        # Something stopped the approach, most likely the hand: back out the
        # way it came and stop, rather than trying another angle at the person.
        with arm.phase("back_out"):
            arm.open_gripper(settle_s=0.0)
            arm.check_abort()
            arm.cartesian_approach(
                to_arm.apply(-approach), PREGRASP_DISTANCE, APPROACH_SPEED
            )
        return _failed(log, "approach to the handle blocked; backed out")

    with arm.phase("wait_for_payload"):
        payload = arm.wait_for_payload(
            PAYLOAD_EFFORT_DELTA, request.release_timeout, list(PAYLOAD_JOINTS)
        )
    if not payload:
        # The person may still be holding it: moving now could pull the bag
        # out of their hand. Stay put and let the caller confirm.
        message = "handle gripped, but no payload: the person may still hold the bag"
        log.warn(message)
        return TakeFromHandOutcome(
            success=True, payload_detected=False, message=message
        )

    arm.check_abort()
    with arm.phase("retreat"):
        if not arm.cartesian_approach(
            to_arm.apply(retreat_direction(approach)), RETREAT_DISTANCE, RETREAT_SPEED
        ):
            log.warn("Retreat with the bag did not complete")

    log.info("Bag taken from the person's hand")
    return TakeFromHandOutcome(success=True, payload_detected=True, message="bag taken")


def _locate_handle(arm, request: TakeFromHandRequest):
    """(handle point, arm base, base_link -> link_base rotation), or a failed outcome."""
    log = arm.logger
    wrist = arm.to_frame(request.hand_point, BASE_FRAME)
    if wrist is None:
        return _failed(
            log,
            f"no TF from '{request.hand_point.header.frame_id}' to {BASE_FRAME}",
        )
    base = arm.link_pose(ARM_FRAME, BASE_FRAME)
    if base is None:
        return _failed(log, f"no TF from {ARM_FRAME} to {BASE_FRAME}")

    target = handle_point(_xyz(wrist.point), request.grasp_below_hand)
    arm_base = _xyz(base.pose.position)
    log.info(
        f"Handle target {target.round(3).tolist()} in {BASE_FRAME}, "
        f"{request.grasp_below_hand:.3f} m below the wrist"
    )
    if not within_reach(target, arm_base):
        return _failed(log, f"handle at {target.round(3).tolist()} is out of reach")

    q = base.pose.orientation
    # link_base's orientation maps arm-frame vectors into base_link; invert it.
    to_arm = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv()
    return target, arm_base, to_arm


def _reach_pregrasp(arm, target, arm_base):
    """Move to the pre-grasp of the first usable yaw; its approach, or None."""
    log = arm.logger
    tip_offset = arm.tip_offset("ee_link_offset")
    for yaw in APPROACH_YAWS_DEG:
        approach = approach_direction(target, arm_base, np.radians(yaw))
        grasp = grasp_pose(target, approach, tip_offset)
        with arm.phase(f"pre_grasp/yaw_{yaw:+d}"):
            if arm.endpoint_self_collides(grasp):
                log.info(f"Yaw {yaw:+d} deg: grasp self-collides, skipping")
                continue
            arm.check_abort()
            pre_grasp = offset_along_approach(grasp, -PREGRASP_DISTANCE)
            if arm.move_to_pregrasp(pre_grasp, velocity=PREGRASP_VELOCITY):
                return approach
            log.info(f"Yaw {yaw:+d} deg: pre-grasp unreachable")
    return None


def _xyz(point) -> np.ndarray:
    return np.array([point.x, point.y, point.z], dtype=float)


def _failed(log, message: str) -> TakeFromHandOutcome:
    log.error(f"Take from hand failed: {message}")
    return TakeFromHandOutcome(success=False, payload_detected=False, message=message)
