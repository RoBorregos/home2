"""The pour pipeline, start to finish.

    find the container -> grasp the source object (or use the held one)
    -> lift above the container -> tilt the wrist -> return

Used to be split across PourManager (perception, its own copy of the GPD grasp
loop) and pour_server (motion), with two ROS action round-trips in between. The
grasp loop is gone: this pipeline calls the pick pipeline instead.
"""

import copy
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from frida_constants.manipulation_constants import (
    GRASP_LINK_FRAME,
    POUR_ACCELERATION,
    POUR_VELOCITY,
)
from frida_interfaces.msg import Constraint
from frida_pymoveit2.robots.xarm6 import JOINT_POSITION_LIMITS
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs_py import point_cloud2
from transforms3d.quaternions import quat2mat

from pick_and_place.pipelines import pick as pick_pipeline
from pick_and_place.pipelines.errors import PickHardwareError
from pick_and_place.pipelines.strategies import PickOutcome

# How far above the container rim to hold the source object before tilting.
POUR_HEIGHT_ABOVE_RIM = 0.15
# Debug marker height, just above the rim.
CENTROID_MARKER_ABOVE_RIM = 0.05

# The grasp frame sits this far along local Z from the gripper centre.
GRIPPER_CENTRE_OFFSET = -0.15
# Lateral offset so the lip, not the centre, sits over the container.
LIP_OFFSET = 0.075

# Wrist tilt that actually pours, and the least tilt worth attempting.
POUR_ANGLE = 3.0  # rad
MIN_POUR_ANGLE = 1.0  # rad

# Fallback orientation when the pick reported no pose.
DEFAULT_POUR_QUAT = [0.707, 0.000, 0.707, 0.002]

# The pour path waits longer for a detection and less for a cluster than the
# pick and place paths do; these were its own values before the merge.
DETECT_TIMEOUT = 60.0
CLUSTER_TIMEOUT = 10.0


@dataclass
class PourRequest:
    """Everything the pour needs from the caller."""

    object_name: str = ""
    container_name: str = ""
    object_already_grasped: bool = False


def execute(
    arm, perception, request: PourRequest, strategies
) -> Tuple[bool, PickOutcome]:
    """Pour the source object into the container. Returns (success, outcome)."""
    log = arm.logger
    arm.set_context("pour")
    log.info(
        f"Pouring '{request.object_name}' into '{request.container_name}' "
        f"(already_grasped={request.object_already_grasped})"
    )

    with arm.phase("initial_pose"):
        arm.move_to_named_position("table_stare", velocity=0.3)

    # Check the source object exists before touching the container: locating the
    # container clusters it and wipes the collision scene, so discovering the
    # source is missing afterwards costs a wasted scene reset.
    if not request.object_already_grasped:
        point = perception.locate_object(request.object_name, timeout=DETECT_TIMEOUT)
        if point is None or point.header.frame_id == "":
            log.error(f"Source object {request.object_name} not found")
            return False, PickOutcome()

    container = _locate_container(arm, perception, request.container_name)
    if container is None:
        return False, PickOutcome()
    rim_centre, rim_top = container

    if request.object_already_grasped:
        outcome = _outcome_from_current_pose(arm)
        if outcome is None:
            log.error("Could not read the current EEF pose for the held object")
            return False, PickOutcome()
    else:
        success, outcome = _grasp_source_object(arm, perception, request, strategies)
        if not success:
            return False, outcome

    pour_pose = _build_pour_pose(arm, rim_centre, rim_top, outcome)

    if not _pour_motion(arm, pour_pose, outcome, request.object_already_grasped):
        log.error("Pour motion failed")
        return False, outcome

    # Let the contents finish draining before moving away.
    time.sleep(5)
    with arm.phase("return/table_stare"):
        for _ in range(5):
            if arm.move_to_named_position("table_stare", velocity=0.3):
                break

    log.info("Pour complete")
    return True, outcome


# ======================================================================
# Find the container
# ======================================================================


def _locate_container(arm, perception, container_name: str):
    """Return (centroid, rim height) of the container, or None."""
    log = arm.logger
    with arm.phase("locate_container"):
        point = perception.locate_object(container_name, timeout=DETECT_TIMEOUT)
        if point is None or point.header.frame_id == "":
            log.error(f"Container {container_name} not found")
            return None

        # add_collision_objects=False: the container must not become an obstacle,
        # the arm has to reach over it.
        cluster = perception.cluster_at(
            point, add_collision_objects=False, timeout=CLUSTER_TIMEOUT
        )
        if cluster is None:
            log.error(f"No cluster for container {container_name}")
            return None

        # The cluster call may have added primitives; clear them either way.
        arm.remove_all_collision_objects(attached=True)

        points = [
            list(p)
            for p in point_cloud2.read_points(
                cluster, field_names=("x", "y", "z"), skip_nans=True
            )
        ]
        if not points:
            log.error("Empty container cluster")
            return None

        array = np.array(points)
        centroid = np.mean(array, axis=0)
        rim_top = float(np.max(array[:, 2]))
        log.info(f"Container centroid: {centroid}, rim top: {rim_top:.3f}")

        marker = PointStamped()
        marker.header.frame_id = cluster.header.frame_id
        marker.header.stamp = arm.now()
        marker.point.x = float(centroid[0])
        marker.point.y = float(centroid[1])
        marker.point.z = rim_top + CENTROID_MARKER_ABOVE_RIM
        arm.publish_debug_point(marker)

        return marker, rim_top


# ======================================================================
# Get the source object into the gripper
# ======================================================================


def _grasp_source_object(arm, perception, request: PourRequest, strategies):
    """Pick the source object using the normal pick pipeline.

    This replaces PourManager's own copy of the GPD grasp loop, which had
    drifted from PickManager's (different sort, different retry count).
    """
    pick_request = pick_pipeline.PickRequest(
        object_name=request.object_name,
        in_configuration=True,  # already at table_stare
        # The pour lifts from wherever the grasp ended. Returning to a carry
        # pose first would send the arm to table_stare and make the lift plan
        # back down toward the surface it just left.
        return_to_carry=False,
    )
    with arm.phase("grasp_source_object"):
        return pick_pipeline.execute(arm, perception, pick_request, strategies)


def _outcome_from_current_pose(arm) -> Optional[PickOutcome]:
    """Synthesise an outcome from where the gripper already is.

    Used when the object is already held, so the pour can lift from the current
    arm position instead of a remembered grasp.
    """
    try:
        transform = arm.tf_buffer.lookup_transform(
            "base_link", GRASP_LINK_FRAME, rclpy.time.Time()
        )
    except Exception as exc:
        arm.logger.error(f"TF lookup failed: {exc}")
        return None

    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp = arm.now()
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return PickOutcome(pick_pose=pose)


# ======================================================================
# Pour
# ======================================================================


def _build_pour_pose(arm, rim_centre, rim_top: float, outcome: PickOutcome):
    """Hold the source object above the container, keeping the grasp orientation."""
    pose = PoseStamped()
    pose.header.frame_id = rim_centre.header.frame_id
    pose.header.stamp = arm.now()
    pose.pose.position.x = rim_centre.point.x
    pose.pose.position.y = rim_centre.point.y
    pose.pose.position.z = rim_top + POUR_HEIGHT_ABOVE_RIM

    if outcome.pick_pose is not None:
        pose.pose.orientation = outcome.pick_pose.pose.orientation
    else:
        pose.pose.orientation.x = DEFAULT_POUR_QUAT[0]
        pose.pose.orientation.y = DEFAULT_POUR_QUAT[1]
        pose.pose.orientation.z = DEFAULT_POUR_QUAT[2]
        pose.pose.orientation.w = DEFAULT_POUR_QUAT[3]
    return pose


def _pour_motion(arm, pour_pose, outcome: PickOutcome, already_grasped: bool) -> bool:
    """Lift clear, move over the container, then tilt the wrist."""
    log = arm.logger
    try:
        if not already_grasped:
            # Lift straight up first: the object was just grasped on a surface and
            # moving sideways at that height would drag it.
            _lift_clear(arm, outcome)
        else:
            log.info("Object already grasped, skipping the lift step")

        # Constraints keep the container upright while traversing, but cost planning
        # time; a held object is already upright so they are skipped.
        use_constraint = not already_grasped

        pose = copy.deepcopy(pour_pose)
        upside_down = _is_upside_down(pose)
        log.info(f"Gripper upside down: {upside_down}")

        pose = _offset_along_local_axis(pose, axis=2, distance=GRIPPER_CENTRE_OFFSET)
        pose = _offset_along_local_axis(
            pose, axis=1, distance=-LIP_OFFSET if upside_down else LIP_OFFSET
        )

        if not _reach_with_retries(arm, pose, use_constraint, tries=5):
            log.error("Could not reach the pour pose")
            return False

        time.sleep(3.0)
        return _tilt_to_pour(arm, pour_pose, pose)
    except PickHardwareError:
        # A motion fault mid-pour leaves the object clamped; release it so the
        # arm can be recovered by hand.
        log.error("Motion fault during the pour; opening the gripper")
        arm.open_gripper()
        raise


def _lift_clear(arm, outcome: PickOutcome) -> bool:
    """Raise the grasped object above the surface before traversing."""
    if outcome.pick_pose is None:
        arm.logger.warn("No pick pose to lift from; skipping the lift")
        return True

    pose = copy.deepcopy(outcome.pick_pose)
    pose.pose.position.z += 0.2
    with arm.phase("lift_clear"):
        for _ in range(2):
            arm.check_abort()
            pose.pose.position.z += 0.025
            if arm.move_to_pose(
                pose,
                velocity=POUR_VELOCITY,
                acceleration=POUR_ACCELERATION,
                tolerance_position=0.04,
                tolerance_orientation=0.4,
                planning_time=10.0,
                planning_attempts=100,
                constraint=_upright_constraint(pose),
            ):
                arm.logger.info("Lifted clear")
                return True
            arm.logger.warn("Lift attempt failed, raising the target")
    # The original continued to the pour even if the lift never succeeded.
    arm.logger.error("Could not lift clear; continuing to the pour anyway")
    return True


def _reach_with_retries(arm, pose, use_constraint: bool, tries: int) -> bool:
    """Try the pour pose, raising it slightly each time."""
    with arm.phase("move_over_container"):
        for attempt in range(tries):
            arm.check_abort()
            pose.pose.position.z += 0.025
            if arm.move_to_pose(
                pose,
                velocity=POUR_VELOCITY,
                acceleration=POUR_ACCELERATION,
                tolerance_position=0.04,
                tolerance_orientation=0.4,
                planning_time=10.0,
                planning_attempts=100,
                constraint=_upright_constraint(pose) if use_constraint else None,
            ):
                arm.logger.info("Pour pose reached")
                return True
            arm.logger.warn(f"Pour pose attempt {attempt + 1}/{tries} failed")
    return False


def _tilt_to_pour(arm, container_pose, gripper_pose) -> bool:
    """Rotate joint 6 so the contents fall towards the container."""
    log = arm.logger
    lower_limit, upper_limit = JOINT_POSITION_LIMITS["joint6"]

    direction = _pour_direction(log, container_pose, gripper_pose)

    joints = arm.get_joints(degrees=False)
    current = joints["joints"]["joint6"]
    target = max(lower_limit, min(upper_limit, current + direction * POUR_ANGLE))

    available = abs(target - current)
    if available < MIN_POUR_ANGLE:
        log.error(
            f"Cannot pour enough in the correct direction (only {available:.2f} rad "
            f"available, need {MIN_POUR_ANGLE}); joint6 is too close to its limit"
        )
        return False

    joints["joints"]["joint6"] = target
    with arm.phase("tilt"):
        if not arm.move_joints(joints, velocity=POUR_VELOCITY):
            log.error("Failed to tilt the wrist")
            return False
    log.info("Poured")
    return True


def _pour_direction(log, container_pose, gripper_pose) -> float:
    """Which way to rotate joint 6 so the contents go into the container.

    A positive joint6 rotation tilts the contents along -local_X, so if -local_X
    points at the container, pour positive.

    PINNED to +1.0 below. Before the pipelines were merged the container pose
    and the gripper pose were the same object, so the distance was always 0 and
    this always fell through to +1.0 -- every pour that has run on this robot
    used the positive direction. The computed sign is therefore untested, and
    getting it wrong tips the container away from the bowl. Validate on
    hardware, then delete this early return.
    """
    return 1.0

    to_container = np.array(
        [
            container_pose.pose.position.x - gripper_pose.pose.position.x,
            container_pose.pose.position.y - gripper_pose.pose.position.y,
            0.0,
        ]
    )
    distance = np.linalg.norm(to_container)

    quat = [
        gripper_pose.pose.orientation.x,
        gripper_pose.pose.orientation.y,
        gripper_pose.pose.orientation.z,
        gripper_pose.pose.orientation.w,
    ]
    local_x = quat2mat(quat)[:, 0].copy()
    local_x[2] = 0.0
    local_x_norm = np.linalg.norm(local_x)

    if distance <= 1e-3 or local_x_norm <= 1e-3:
        log.warn(
            f"Could not determine the pour direction (distance={distance:.4f}, "
            f"local_x_norm={local_x_norm:.4f}); defaulting to positive"
        )
        return 1.0

    dot = float(np.dot(-local_x / local_x_norm, to_container / distance))
    log.info(f"Pour direction {'positive' if dot >= 0 else 'negative'} (dot={dot:.3f})")
    return 1.0 if dot >= 0 else -1.0


def _upright_constraint(pose):
    """Keep the gripper's orientation while the arm traverses."""
    constraint = Constraint()
    constraint.orientation = pose.pose.orientation
    constraint.frame_id = pose.header.frame_id
    constraint.target_link = GRASP_LINK_FRAME
    # Free about the approach axis, tight about the other two: the container may
    # spin in the gripper but must not tip.
    constraint.tolerance_orientation = [3.14 * 2, 2.0, 2.0]
    constraint.weight = 1.0
    constraint.parameterization = 0
    return constraint


def _is_upside_down(pose) -> bool:
    """True when the gripper's local X points upward.

    Project a point along local X: if it lands above the pose, the gripper is
    the other way up and the lip offset must flip.
    """
    quat = [
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ]
    x_axis = quat2mat(quat)[:, 0]
    projected_z = pose.pose.position.z + x_axis[2] * 0.15
    return projected_z >= pose.pose.position.z


def _offset_along_local_axis(pose, axis: int, distance: float):
    """Copy ``pose`` translated ``distance`` along one of its local axes."""
    shifted = copy.deepcopy(pose)
    quat = [
        shifted.pose.orientation.x,
        shifted.pose.orientation.y,
        shifted.pose.orientation.z,
        shifted.pose.orientation.w,
    ]
    direction = quat2mat(quat)[:, axis]
    position = (
        np.array(
            [
                shifted.pose.position.x,
                shifted.pose.position.y,
                shifted.pose.position.z,
            ]
        )
        + direction * distance
    )
    shifted.pose.position.x = float(position[0])
    shifted.pose.position.y = float(position[1])
    shifted.pose.position.z = float(position[2])
    return shifted


__all__ = ["PourRequest", "execute"]
