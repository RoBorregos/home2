"""The place pipeline, start to finish.

choose where to put it -> build a ladder of candidate poses
-> reach one -> release -> return

"""

import copy
import json
from typing import List, Optional

import numpy as np
from frida_constants.manipulation_constants import (
    AIM_STRAIGHT_FRONT_QUAT,
    EEF_LINK_NAME,
    GRASP_LINK_FRAME,
    SHELF_MIN_REACH,
    SHELF_POSITION_PREPLACE_POSE,
    SHELF_REACH_BASE,
    SHELF_REACH_SLOPE,
    TRASH_BIN_NAME,
)
from frida_interfaces.srv import HeatmapPlace, PlacePerceptionService
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_motion_planning.utils.tf_utils import transform_point, transform_pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2
from transforms3d.quaternions import quat2mat

from pick_and_place.pipelines.strategies import PickOutcome
from pick_and_place.robot.geometry import set_quaternion

# Shelf place: how far (m) to pull the place toward the front edge so the arm
# and eye-in-hand camera do not jam against the short compartment ceiling.
SHELF_PLACE_DEPTH_BACK = 0.07

# Ladder of candidate place heights: fewer, finer steps on a shelf because the
# compartment ceiling is close.
SHELF_LADDER = (3, 0.02)
TABLE_LADDER = (8, 0.05)

# Fallback drop height when the pick reported no object height.
DEFAULT_SHELF_DROP = 0.15
DEFAULT_TABLE_DROP = 0.10

# The motion planner's own defaults. Pick's tighter ones make every rung of the
# ladder harder to plan.
PLACE_TOLERANCE_POSITION = 0.01
PLACE_TOLERANCE_ORIENTATION = 0.05

# The heatmap ranks a whole surface and the estimator averages several frames;
# both are slow, hence the generous waits.
HEATMAP_TIMEOUT = 60.0
TRASH_TIMEOUT = 15.0

# Orientation with Z aiming down, used when the pick pose is unusable.
TOP_DOWN_QUAT = [0.0, 1.0, 0.0, 0.0]


def execute(arm, perception, place_params, pick_outcome: PickOutcome) -> bool:
    """Place the held object. Returns True on success."""
    log = arm.logger
    arm.set_context("place")
    log.info("Executing place task")

    if not place_params.is_shelf and not place_params.skip_initial_pose:
        with arm.phase("initial_pose"):
            arm.move_to_named_position("table_stare", velocity=0.3)

    with arm.phase("choose_pose"):
        place_pose = _choose_place_pose(arm, perception, place_params, pick_outcome)
    if place_pose is None:
        log.error("Could not determine a place pose")
        return False
    log.info(
        f"Place pose: {place_pose.pose.position.x:.3f}, "
        f"{place_pose.pose.position.y:.3f}, {place_pose.pose.position.z:.3f}"
    )
    arm.publish_place_pose(place_pose)

    reached = _reach_place_pose(arm, place_pose, place_params.is_shelf)

    if reached:
        with arm.phase("release"):
            arm.detach_pick_objects()
            arm.open_gripper(settle_s=1.0)

    # The eye-in-hand octomap never sees the compartment ceiling, so add an
    # explicit slab there before planning the return.
    if place_params.is_shelf:
        arm.add_shelf_ceiling_guard(place_pose, place_params.table_height)
    try:
        with arm.phase("return/table_stare"):
            for _ in range(5):
                if arm.move_to_named_position("table_stare", velocity=0.5):
                    log.info("Returned to position successfully")
                    break
                log.info("Retry sending return joint goal")
    finally:
        # Always drop the guard so it does not constrain later motions.
        if place_params.is_shelf:
            arm.remove_shelf_ceiling_guard()

    return reached


# ======================================================================
# Reach the place pose
# ======================================================================


def _reach_place_pose(arm, place_pose: PoseStamped, is_shelf: bool) -> bool:
    """Try a ladder of increasing heights until one is reachable.

    No retract afterwards (mirroring the shelf pick): the cartesian retract left
    the wrist wound and the following table_stare then aborted.
    """
    log = arm.logger
    count, step = SHELF_LADDER if is_shelf else TABLE_LADDER
    target_link = EEF_LINK_NAME if is_shelf else GRASP_LINK_FRAME

    def reach(pose):
        return arm.move_to_pose(
            pose,
            target_link=target_link,
            tolerance_position=PLACE_TOLERANCE_POSITION,
            tolerance_orientation=PLACE_TOLERANCE_ORIENTATION,
        )

    for index in range(count):
        arm.check_abort()
        final, half, pre = _place_pose_triple(place_pose, index, step, is_shelf)
        arm.set_context(f"place rung={index}")

        if is_shelf:
            with arm.phase("pre_place"):
                if not reach(pre):
                    log.error(f"Failed to reach pre-place pose at rung {index}")
                    continue

        with arm.phase("place"):
            if reach(final):
                log.info(f"Place pose reached at rung {index}")
                return True

        if is_shelf and index == 0:
            # Halfway only on the first rung; higher rungs are already elevated
            # and a halfway pose there risks a ceiling collision.
            with arm.phase("place_halfway"):
                if reach(half):
                    log.info("Place halfway pose reached")
                    return True
        elif is_shelf:
            log.info(f"Skipping halfway fallback at rung {index} (already elevated)")

    log.error("Failed to reach any place pose")
    return False


def _place_pose_triple(
    place_pose: PoseStamped, index: int, step: float, is_shelf: bool
):
    """Build (final, halfway, pre-place) for one rung of the ladder.

    On a table the three are identical; only the shelf needs an approach from
    further out along the gripper's local Z.
    """
    final = copy.deepcopy(place_pose)
    final.pose.position.z += index * step

    if not is_shelf:
        return final, copy.deepcopy(final), copy.deepcopy(final)

    # The heatmap aims at the compartment center (deep), which jams the arm and
    # camera against the ceiling; pull the place toward the front edge.
    final.pose.position.x -= SHELF_PLACE_DEPTH_BACK
    set_quaternion(final, AIM_STRAIGHT_FRONT_QUAT)

    pre = _offset_along_local_z(final, SHELF_POSITION_PREPLACE_POSE)
    half = _offset_along_local_z(final, SHELF_POSITION_PREPLACE_POSE * 0.5)
    return final, half, pre


def _offset_along_local_z(pose: PoseStamped, distance: float) -> PoseStamped:
    """Copy ``pose`` translated ``distance`` along its own local Z axis."""
    shifted = copy.deepcopy(pose)
    quat = [
        shifted.pose.orientation.w,
        shifted.pose.orientation.x,
        shifted.pose.orientation.y,
        shifted.pose.orientation.z,
    ]
    z_axis = quat2mat(quat)[:, 2]
    position = (
        np.array(
            [shifted.pose.position.x, shifted.pose.position.y, shifted.pose.position.z]
        )
        + z_axis * distance
    )
    shifted.pose.position.x = float(position[0])
    shifted.pose.position.y = float(position[1])
    shifted.pose.position.z = float(position[2])
    return shifted


# ======================================================================
# Choose where to place
# ======================================================================


def _choose_place_pose(
    arm, perception, place_params, pick_outcome: PickOutcome
) -> Optional[PoseStamped]:
    """Decide where the object goes, then lift it clear of the surface."""
    if place_params.is_trash:
        return _trash_pose(perception)

    if place_params.forced_pose.header.frame_id != "":
        pose = _forced_pose(arm, place_params, pick_outcome)
    else:
        pose = _heatmap_pose(arm, perception, place_params)

    if pose is None:
        return None
    return _apply_drop_height(arm, pose, place_params, pick_outcome)


def _forced_pose(arm, place_params, pick_outcome: PickOutcome) -> Optional[PoseStamped]:
    """Use the caller's pose, transformed into the pick's frame."""
    arm.logger.info("Using forced place pose")
    # Prefer the pick's frame so the orientation carries over correctly;
    # base_link otherwise.
    frame = "base_link"
    if pick_outcome.pick_pose is not None and pick_outcome.pick_pose.header.frame_id:
        frame = pick_outcome.pick_pose.header.frame_id

    arm.logger.info(f"Transforming forced place pose to {frame}")
    success, pose = transform_pose(place_params.forced_pose, frame, arm.tf_buffer)
    if not success:
        arm.logger.error("Failed to transform forced place pose")
        return None
    return pose


def _heatmap_pose(arm, perception, place_params) -> Optional[PoseStamped]:
    """Ask the heatmap service for the best free spot on the surface."""
    log = arm.logger
    request = HeatmapPlace.Request()

    # "Place next to X": aim the heatmap at X's centroid.
    if place_params.close_to:
        point = _close_by_point(arm, perception, place_params.close_to)
        if point is not None:
            request.close_point = point
        else:
            log.warn(
                f"Could not resolve close_to='{place_params.close_to}'; "
                "placing on any free spot instead"
            )

    # A special request can instead put the object directly on top of another.
    on_top = None
    if place_params.special_request:
        on_top, close_point = _special_request(arm, perception, place_params)
        if close_point is not None:
            request.close_point = close_point
            request.special_request = place_params.special_request
        elif on_top is None:
            log.warn(
                f"special_request {place_params.special_request!r} yielded no target; "
                "placing on any free spot instead"
            )
    if on_top is not None:
        return on_top

    cloud = _place_surface_cloud(arm, perception, place_params)
    if cloud is None:
        return None
    request.pointcloud = cloud

    # Tables bias toward the nearest reachable point; only shelves opt out.
    # High shelves are the exception to the exception: the arm can barely reach
    # the center, so prefer the nearest point. Low/mid shelves want the densest center area, not the
    # front edge, which causes edge drops.
    request.prefer_closest = (
        place_params.table_height > 1.0 if place_params.is_shelf else True
    )

    if not perception.heatmap_place_client.wait_for_service(timeout_sec=5.0):
        log.error("place_pose (heatmap) service not available")
        return None
    future = perception.heatmap_place_client.call_async(request)
    if not wait_for_future(future, timeout=HEATMAP_TIMEOUT) or future.result() is None:
        log.error("heatmap place service did not respond")
        return None
    point = future.result().place_point
    log.info(
        f"Place point detected: {point.point.x:.3f}, "
        f"{point.point.y:.3f}, {point.point.z:.3f}"
    )

    if place_params.is_shelf:
        _clamp_to_reach(log, point, place_params.table_height)

    pose = PoseStamped()
    pose.header = point.header
    pose.pose.position = point.point
    return pose


def _close_by_point(arm, perception, object_name: str):
    """Locate an object and return its cluster centroid in base_link."""
    log = arm.logger
    point = None
    centroid_found = False
    for attempt in range(5):
        try:
            log.info(f"Locating '{object_name}' to place near it")
            point = perception.locate_object(object_name)
            if point is None:
                log.warn(f"Failed to get close-by point for {object_name}, retrying")
                continue
            cluster = perception.cluster_at(point, add_collision_objects=False)
            if cluster is None:
                log.warn(f"No cluster for {object_name} ({attempt + 1}/5), retrying")
                continue
            log.info(f"Object cluster detected for {object_name}")
            arm.remove_all_collision_objects(attached=False)
            centroid = _centroid(cluster)
            if centroid is None:
                log.warn(f"Empty cluster for {object_name} ({attempt + 1}/5), retrying")
                continue
            point.point.x = float(centroid[0])
            point.point.y = float(centroid[1])
            point.point.z = float(centroid[2])
            point.header.frame_id = cluster.header.frame_id
            centroid_found = True
            break
        except Exception as exc:
            log.error(f"Failed to get object: {exc}")

    if point is not None and not centroid_found:
        log.warn(
            f"No cluster centroid for {object_name} after 5 tries; "
            "using the raw detection point instead"
        )
    return _to_base_link(arm, point, object_name)


def _locate_point(arm, perception, object_name: str):
    """Locate an object by label, in base_link. No clustering.

    The special-request path places relative to the detected point; clustering
    here would also wipe the collision scene mid-place.
    """
    log = arm.logger
    point = None
    for _ in range(5):
        try:
            point = perception.locate_object(object_name)
            if point is not None:
                break
            log.warn(f"Failed to locate {object_name}, retrying")
        except Exception as exc:
            log.error(f"Failed to get object: {exc}")
    return _to_base_link(arm, point, object_name)


def _to_base_link(arm, point, object_name: str):
    if point is None:
        arm.logger.error(f"Could not locate {object_name}")
        return None
    success, point = transform_point(point, "base_link", arm.tf_buffer)
    if not success:
        arm.logger.error("Failed to transform point to base_link")
        return None
    arm.logger.info(f"Located {object_name} at {point.point}")
    return point


def _special_request(arm, perception, place_params):
    """Handle the JSON special request. Returns (on_top_pose, close_point)."""
    log = arm.logger
    try:
        parsed = json.loads(place_params.special_request)
    except (ValueError, TypeError) as exc:
        log.error(f"Could not parse special_request: {exc}")
        return None, None

    kind = parsed.get("request", "")
    if kind != "close_by":
        log.warn(f"Unsupported special_request {kind!r}; ignoring it")
        return None, None

    log.info("Using close-by place pose")
    # Detection only -- no clustering, no centroid, no collision wipe.
    point = _locate_point(arm, perception, parsed.get("object", ""))
    if point is None:
        return None, None

    if parsed.get("location", "") == "top":
        pose = PoseStamped()
        pose.header.frame_id = point.header.frame_id
        pose.pose.position.x = point.point.x
        pose.pose.position.y = point.point.y
        pose.pose.position.z = point.point.z
        return pose, None

    if point.header.frame_id:
        return None, point
    log.warn(
        f"'{parsed.get('object', '')}' resolved without a frame; "
        "ignoring the special request"
    )
    return None, None


def _place_surface_cloud(arm, perception, place_params):
    """Segment the table or shelf surface the object will land on."""
    log = arm.logger
    log.info("Extracting the place surface cloud")
    request = PlacePerceptionService.Request()
    request.place_params = place_params
    if not perception.place_perception_client.wait_for_service(timeout_sec=5.0):
        log.error("place_perception service not available")
        return None
    future = perception.place_perception_client.call_async(request)
    if not wait_for_future(future, timeout=10) or future.result() is None:
        log.error("place_perception service did not respond")
        return None
    cloud = future.result().cluster_result
    if len(cloud.data) == 0:
        log.error("No plane cluster detected")
        return None
    log.info(f"Plane cluster detected: {len(cloud.data)} points")
    return cloud


def _clamp_to_reach(log, point, shelf_z: float) -> None:
    """Clamp X to the arm's reachable envelope for this shelf height."""
    max_x = max(SHELF_MIN_REACH, SHELF_REACH_BASE - SHELF_REACH_SLOPE * shelf_z)
    if point.point.x > max_x:
        original = point.point.x
        point.point.x = max_x
        log.info(
            f"Clamped shelf place X from {original:.3f} to {max_x:.3f} "
            f"(shelf_z={shelf_z:.3f})"
        )


def _apply_drop_height(
    arm, pose: PoseStamped, place_params, pick_outcome: PickOutcome
) -> PoseStamped:
    """Lift the place point by how high the object was grasped, and orient it."""
    log = arm.logger
    pick_pose = pick_outcome.pick_pose
    unknown = (
        pick_outcome.object_pick_height == 0
        or pick_pose is None
        or pick_pose.header.frame_id == ""
    )

    if unknown:
        log.warn("No object height from the pick; using the default drop height")
        drop = DEFAULT_SHELF_DROP if place_params.is_shelf else DEFAULT_TABLE_DROP
        orientation_source = set_quaternion(PoseStamped(), TOP_DOWN_QUAT)
    else:
        log.info(f"Object pick height: {pick_outcome.object_pick_height}")
        drop = pick_outcome.object_pick_height
        orientation_source = pick_pose

    # On a shelf the compartment ceiling, not the object, sets the clearance.
    pose.pose.position.z += 0.1 if place_params.is_shelf else drop
    pose.pose.orientation = orientation_source.pose.orientation
    return pose


def _trash_pose(perception) -> Optional[PoseStamped]:
    """Detect the trash bin; the estimator already offsets above it, top-down."""
    response = perception.estimate_flat_grasp(TRASH_BIN_NAME, timeout=TRASH_TIMEOUT)
    if response is None:
        perception.logger.error("Trash bin detection failed")
        return None
    perception.logger.info(
        f"Trash place pose: {response.pose.pose.position.x:.3f}, "
        f"{response.pose.pose.position.y:.3f}, {response.pose.pose.position.z:.3f}"
    )
    return response.pose


def _centroid(cluster) -> Optional[List[float]]:
    points = [
        list(p)
        for p in point_cloud2.read_points(
            cluster, field_names=("x", "y", "z"), skip_nans=True
        )
    ]
    if not points:
        return None
    return np.mean(np.array(points), axis=0)


__all__ = ["execute"]
