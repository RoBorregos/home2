"""The pick pipeline, start to finish.

    look at the object -> perceive it -> find grasp candidates
    -> run the motion strategy -> return to a carry pose

Read this file top to bottom and you have the whole pick.
"""

import copy
import time
from dataclasses import dataclass, field
from typing import Iterator, List, Optional, Tuple

import numpy as np
from frida_constants.manipulation_constants import PICK_MAX_DISTANCE
from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs_py import point_cloud2

from pick_and_place.pipelines.classification import (
    PICK_STRATEGY_FLAT,
    PICK_STRATEGY_GPD,
    PICK_STRATEGY_PEAK,
    PICK_STRATEGY_RIM,
    resolve_pick_strategy,
)
from pick_and_place.pipelines.errors import PickAttemptFailed
from pick_and_place.pipelines.strategies import GraspCandidate, PickOutcome
from pick_and_place.robot.geometry import offset_along_approach
from pick_and_place.utils.grasp_orientation import is_frontal_grasp

# GPD configs, tried in order until one yields a graspable candidate. The bool
# marks a config whose grasps are reversible (a 180 deg flip is also valid).
CFG_PATHS = [
    [
        "/workspace/src/manipulation/packages/arm_pkg/config/"
        "frida_eigen_params_custom_gripper_testing.cfg",
        True,
    ],
    [
        "/workspace/src/manipulation/packages/arm_pkg/config/"
        "frida_eigen_params_custom_gripper.cfg",
        False,
    ],
]

# Shelf picks use a frontal-biased cfg with a wider approach cone (the table
# 30 deg cone over-prunes the oblique shelf view).
SHELF_CFG_PATHS = [
    [
        "/workspace/src/manipulation/packages/arm_pkg/config/"
        "frida_eigen_params_custom_gripper_shelf.cfg",
        False,
    ],
]

# Fine height adjustment for flat objects. Negative if the grasp floats,
# positive if it hits too hard.
FLAT_GRASP_Z_TWEAK = 0.076

# Objects shorter than this cannot be grasped from a reversed approach.
MIN_REVERSIBLE_HEIGHT = 0.06

# The ZED hangs below the gripper and grazes the shelf surface on a low frontal
# grasp; the offset is used to decide whether flipping lifts the camera clear.
ZED_OFFSET = np.array([-0.096, 0.0, 0.041])

# Where to look for each kind of object, before perceiving it. Every flat object
# uses the same pose; anything not listed (bowl, GPD) uses the default.
STARE_POSES = {
    PICK_STRATEGY_RIM: "look_side_stare",
    PICK_STRATEGY_PEAK: "look_side_stare",
    PICK_STRATEGY_FLAT: "flat_stare",
}
DEFAULT_STARE_POSE = "table_stare"


@dataclass
class PickRequest:
    """Everything the pick needs from the caller."""

    object_name: str = ""
    object_point: Optional[PointStamped] = None
    min_distance: float = 0.0
    max_distance: float = PICK_MAX_DISTANCE
    is_shelf: bool = False
    in_configuration: bool = False
    # False leaves the arm where the grasp ended, for callers that move on from
    # there themselves (the pour lifts straight up from the grasp).
    return_to_carry: bool = True


@dataclass
class GraspSet:
    """One batch of grasp candidates to try before falling back to the next."""

    poses: List = field(default_factory=list)
    scores: List[float] = field(default_factory=list)
    source: str = ""


@dataclass
class Perceived:
    """What perception found, handed to grasp generation."""

    cluster: Optional[object] = None
    height: float = 0.0
    flat_pose: Optional[object] = None


def execute(
    arm, perception, request: PickRequest, strategies
) -> Tuple[bool, PickOutcome]:
    """Pick an object. Returns (success, outcome)."""
    log = arm.logger
    strategy_key = resolve_pick_strategy(request.object_name)
    strategy = strategies[strategy_key]
    arm.set_context(strategy_key)

    log.info(
        f"[{strategy_key}] picking '{request.object_name}' (shelf={request.is_shelf})"
    )

    if not request.in_configuration:
        _move_to_stare_pose(arm, strategy_key)

    perceived = _perceive(perception, request, strategy_key)
    if perceived is None:
        return False, PickOutcome()

    # Open after perceiving: the fingers are in the camera's view while it looks.
    arm.open_gripper()

    # Remember the perceived objects so DirectGraspPick can attach one.
    arm.snapshot_scene()

    outcome = None
    for grasp_set in _grasp_sets(perception, perceived, request, strategy_key):
        outcome = _run_strategy(arm, strategy, grasp_set)
        if outcome is not None:
            break
        time.sleep(0.2)

    if outcome is None:
        log.error(f"[{strategy_key}] pick failed: no candidate succeeded")
        return False, PickOutcome()

    # Defensive re-close: the strategy already closed, but a missed service
    # response there would otherwise leave the object un-gripped for the return.
    arm.close_gripper(settle_s=0.0)

    outcome.object_name = request.object_name
    if request.return_to_carry:
        _return_to_carry_pose(arm, strategy_key, request.is_shelf)
    log.info(f"[{strategy_key}] pick complete")
    return True, outcome


# ======================================================================
# Look at the object
# ======================================================================


def _move_to_stare_pose(arm, strategy_key: str) -> None:
    """Point the camera where this kind of object lives."""
    named = STARE_POSES.get(strategy_key, DEFAULT_STARE_POSE)
    with arm.phase(f"stare/{named}"):
        arm.move_to_named_position(named, velocity=0.75)


# ======================================================================
# Perceive
# ======================================================================


def _perceive(
    perception, request: PickRequest, strategy_key: str
) -> Optional[Perceived]:
    """Locate the object. Returns None when it cannot be found."""
    if strategy_key != PICK_STRATEGY_GPD:
        return _perceive_flat(perception, request, strategy_key)
    return _perceive_cluster(perception, request)


def _perceive_flat(perception, request: PickRequest, strategy_key: str):
    """Ask the estimator for a top-down pose.

    Deliberately does NOT cluster: clustering adds the table as a collision
    object, which makes MoveIt reject every near-table path.
    """
    perception.logger.info(
        f"Flat object '{request.object_name}': asking the estimator for a pose"
    )
    response = perception.estimate_flat_grasp(request.object_name)
    if response is None:
        return None

    # Rim/peak/bowl poses are used as-is (the strategy applies its own offset);
    # flat objects get the table-tuned tweak.
    z_tweak = FLAT_GRASP_Z_TWEAK if strategy_key == PICK_STRATEGY_FLAT else 0.0
    pose = response.pose
    pose.pose.position.z += z_tweak
    perception.logger.info(
        f"Flat grasp pose received ({response.samples_collected} samples), "
        f"z tweak={z_tweak}"
    )
    return Perceived(flat_pose=pose)


def _perceive_cluster(perception, request: PickRequest) -> Optional[Perceived]:
    """Detect the object and segment it into a cloud."""
    point = _locate(perception, request)
    if point is None:
        return None

    cluster = None
    for _ in range(3):
        cluster = perception.cluster_at(point)
        if cluster is not None:
            break
    if cluster is None:
        perception.logger.error("No object cluster detected")
        return None

    height = _cluster_height(cluster)
    if height is None:
        perception.logger.error("Empty object cluster")
        return None
    perception.logger.info(f"Object cluster height: {height:.2f} m")
    return Perceived(cluster=cluster, height=height)


def _locate(perception, request: PickRequest) -> Optional[PointStamped]:
    """Resolve the object to a 3D point, by given point or by label."""
    point = request.object_point
    if point is not None and (point.point.x or point.point.y or point.point.z):
        perception.logger.info(f"Going for the given point: {point.point}")
        return point

    if not request.object_name:
        perception.logger.error("No object name or point provided")
        return None

    perception.logger.info(f"Going for object name: {request.object_name}")
    point = perception.locate_object(request.object_name)
    if point is None or point.header.frame_id == "":
        perception.logger.error(f"Object {request.object_name} not found")
        return None

    distance = (point.point.x**2 + point.point.y**2 + point.point.z**2) ** 0.5
    if not perception.point_in_range(point, request.min_distance, request.max_distance):
        perception.logger.error(
            f"Object {request.object_name} is out of range (distance {distance:.2f} m, "
            f"expected [{request.min_distance}, {request.max_distance}] m)"
        )
        return None
    if point.point.z > 1.0:
        perception.logger.error(f"Object {request.object_name} is too far")
        return None
    return point


def _cluster_height(cluster) -> Optional[float]:
    points = [
        list(p)
        for p in point_cloud2.read_points(
            cluster, field_names=("x", "y", "z"), skip_nans=True
        )
    ]
    if not points:
        return None
    z = np.array(points)[:, 2]
    return float(np.max(z) - np.min(z))


# ======================================================================
# Find grasp candidates
# ======================================================================


def _grasp_sets(
    perception, perceived: Perceived, request: PickRequest, strategy_key: str
) -> Iterator[GraspSet]:
    """Yield batches of candidates, best source first.

    Flat objects yield a single set from the estimator. GPD yields one set per
    config, so a config whose grasps are all unreachable falls back to the next.
    """
    if strategy_key != PICK_STRATEGY_GPD:
        # A 90 deg alternative would align the fingers with a fork's long axis
        # and collide with it, so the only alternative is the symmetric flip.
        yield GraspSet(
            poses=[
                perceived.flat_pose,
                _rotate_about_approach(perceived.flat_pose, 180),
            ],
            scores=[1.0, 0.9],
            source="flat_estimator",
        )
        return

    # Shelf: retry the same config a few times. GPD samples randomly, so
    # re-detecting recovers a round where every grasp was unreachable.
    cfg_list = SHELF_CFG_PATHS * 3 if request.is_shelf else CFG_PATHS

    for cfg_path, is_reversible in cfg_list:
        if is_reversible and perceived.height < MIN_REVERSIBLE_HEIGHT:
            perception.logger.warn(
                "Object too small for reversible grasping, skipping that config"
            )
            continue

        poses, scores = perception.detect_grasps(perceived.cluster, cfg_path)
        if not poses:
            continue

        poses, scores = _select_candidates(poses, scores, request.is_shelf)
        if is_reversible:
            poses, scores = _add_reversed(poses, scores)
        if not poses:
            perception.logger.error("No grasp poses survived selection")
            continue

        if request.is_shelf:
            poses, scores = _prefer_frontal(
                poses, scores, request.object_name, perception.logger
            )
            _lift_camera_clear(poses)

        yield GraspSet(poses=poses, scores=scores, source=cfg_path.rsplit("/", 1)[-1])


def _select_candidates(poses, scores, is_shelf: bool):
    """Trim the GPD output to a handful of candidates worth trying."""
    if is_shelf:
        # Keep the highest-scored candidates (not a random 5) and try more of
        # them, so a reachable, higher-quality grasp wins over a marginal one.
        order = list(np.argsort(scores)[::-1][:8])
        return [poses[i] for i in order], [scores[i] for i in order]
    if len(poses) > 5:
        indices = np.random.choice(len(poses), size=5, replace=False)
        return [poses[i] for i in indices], [scores[i] for i in indices]
    return poses[:5], scores[:5]


def _add_reversed(poses, scores):
    """Interleave each grasp with its 180 deg twin, capped at 5."""
    out_poses = []
    out_scores = []
    for pose, score in zip(poses, scores):
        out_poses.append(pose)
        out_scores.append(score)
        out_poses.append(_rotate_about_approach(pose, 180))
        out_scores.append(score)
    return out_poses[:5], out_scores[:5]


def _prefer_frontal(poses, scores, object_name: str, log):
    """Shelf picks want a frontal grasp; top-down hits the compartment ceiling."""
    kept = [
        (pose, score)
        for pose, score in zip(poses, scores)
        if is_frontal_grasp(
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
        )
    ]
    if not kept:
        log.warn(f"Frontal grasp filter removed all for {object_name}; using all")
        return poses, scores
    log.info(f"Frontal grasp filter kept {len(kept)}/{len(poses)} for {object_name}")
    return [k[0] for k in kept], [k[1] for k in kept]


def _lift_camera_clear(poses) -> None:
    """Flip about the approach axis when that lifts the ZED off the shelf.

    The flip yields the same grasp, so it is free to take whichever orientation
    keeps the camera higher.
    """
    for pose in poses:
        quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        rotation = R.from_quat(quat)
        camera_z = pose.pose.position.z + float(rotation.apply(ZED_OFFSET)[2])
        flipped = (rotation * R.from_euler("z", 180, degrees=True)).as_quat()
        flipped_z = pose.pose.position.z + float(
            R.from_quat(flipped).apply(ZED_OFFSET)[2]
        )
        if flipped_z > camera_z:
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = flipped


def _rotate_about_approach(pose, degrees: float):
    """Copy ``pose`` rotated about its own approach (local Z) axis."""
    rotated = copy.deepcopy(pose)
    original = R.from_quat(
        [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
    )
    result = (original * R.from_euler("z", degrees, degrees=True)).as_quat()
    rotated.pose.orientation.x = result[0]
    rotated.pose.orientation.y = result[1]
    rotated.pose.orientation.z = result[2]
    rotated.pose.orientation.w = result[3]
    return rotated


# ======================================================================
# Run the motion strategy
# ======================================================================


def _run_strategy(arm, strategy, grasp_set: GraspSet):
    """Try every candidate in ``grasp_set``. Returns an outcome or None."""
    log = arm.logger
    profile = strategy.profile
    total = len(grasp_set.poses) * profile.num_alternatives
    log.info(
        f"[{strategy.name}] trying {len(grasp_set.poses)} pose(s) x "
        f"{profile.num_alternatives} alternative(s) = {total} candidate(s) "
        f"from {grasp_set.source}"
    )

    failures = []
    for candidate in _candidates(arm, profile, grasp_set):
        arm.check_abort()
        arm.set_context(f"{strategy.name} {candidate.label()}")
        try:
            return strategy.attempt(arm, candidate)
        except PickAttemptFailed as exc:
            log.warn(f"[{strategy.name} {candidate.label()}] {exc}")
            failures.append(f"{candidate.label()}: {exc}")

    # One line carrying every attempt, so a failed pick is diagnosable without
    # scrolling back through the whole run.
    log.error(
        f"[{strategy.name}] all {len(failures)} candidate(s) from "
        f"{grasp_set.source} failed: " + "; ".join(failures)
    )
    return None


def _candidates(arm, profile, grasp_set: GraspSet) -> Iterator[GraspCandidate]:
    """Enumerate candidates with the tip offset already applied.

    Every strategy needs the perceived grasp point backed off along the
    gripper's approach axis to the EE link frame; strategies that want extra
    attempts get them by stepping that offset.
    """
    base_offset = arm.tip_offset(profile.tip_offset_param)
    for pose_index, pose in enumerate(grasp_set.poses):
        score = (
            grasp_set.scores[pose_index] if pose_index < len(grasp_set.scores) else 0.0
        )
        for alternative in range(profile.num_alternatives):
            offset = base_offset + alternative * profile.alternative_step
            yield GraspCandidate(
                pose=offset_along_approach(pose, offset),
                score=score,
                pose_index=pose_index,
                alternative_index=alternative,
            )


# ======================================================================
# Return to a carry pose
# ======================================================================


def _return_to_carry_pose(arm, strategy_key: str, is_shelf: bool) -> None:
    """Where the arm goes once it is holding the object."""
    log = arm.logger

    if is_shelf:
        # Retract to front_stare first to clear the compartment ceiling; the
        # strategy-specific return below still runs afterwards.
        log.info("Shelf pick: retracting to front_stare")
        with arm.phase("return/front_stare"):
            arm.move_to_named_position("front_stare", velocity=0.3)

    if strategy_key == PICK_STRATEGY_RIM:
        # Hold where the strategy left the arm; a basket must be carried in place.
        log.info("Rim pick: holding position (skipping return to stare)")
        return

    named = "look_side_stare" if strategy_key == PICK_STRATEGY_PEAK else "table_stare"
    log.info(f"Returning to {named}")
    with arm.phase(f"return/{named}"):
        arm.clear_octomap()
        for _ in range(5):
            if arm.move_to_named_position(named, velocity=0.5):
                return
        log.warn(f"Could not return to {named} after 5 attempts")


__all__ = ["GraspSet", "PickOutcome", "PickRequest", "Perceived", "execute"]
