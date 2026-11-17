"""The pick pipeline, start to finish.

    look at the object -> perceive it -> find grasp candidates
    -> run the motion strategy -> return to a carry pose

Read this file top to bottom and you have the whole pick.
"""

import copy
import time
from dataclasses import dataclass, field, replace
from typing import Iterator, List, Optional, Tuple

import numpy as np
from frida_constants.manipulation_constants import (
    CARTESIAN_PLANNER,
    EEF_LINK_NAME,
    GRIPPER_MAX_APERTURE,
    GRIPPER_REACH,
    PICK_MAX_DISTANCE,
    SAFETY_HEIGHT_GEOMETRIC,
    TOP_GRASP_DEPTH,
)
from frida_motion_planning.utils.tf_utils import transform_point, transform_pose
from geometry_msgs.msg import PointStamped, PoseStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs_py import point_cloud2

from pick_and_place.pipelines.classification import (
    PICK_STRATEGY_FLAT,
    PICK_STRATEGY_GEOMETRIC,
    PICK_STRATEGY_GPD,
    PICK_STRATEGY_PEAK,
    PICK_STRATEGY_RIM,
    resolve_pick_strategy,
)
from pick_and_place.pipelines.errors import PickAttemptFailed
from pick_and_place.pipelines.strategies import GraspCandidate, PickOutcome
from pick_and_place.robot.geometry import (
    approach_axis,
    is_tilted,
    offset_along_approach,
    set_quaternion,
)
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

# Shelf clustering crops to the object's compartment, this far inside its boards.
SHELF_BAND_MARGIN = 0.01
SHELF_BAND_DOWN_TOLERANCE = 0.05  # a detection point may sit this far under its board

# Shelf frontal grasp (fingertip heights, base frame): GPD's tilted grasps do not fit a compartment.
SHELF_FRONTAL_BOARD_CLEARANCE = 0.06  # fingertips this far above the board
SHELF_FRONTAL_CEILING_CLEARANCE = 0.06  # ...and this far under the ceiling
SHELF_FRONTAL_TOP_MARGIN = 0.02  # never above the object's top minus this
SHELF_FRONTAL_Z_STEPS = (0.0, 0.025, -0.025)
SHELF_FRONTAL_WIDTH_MARGIN = 0.01  # the object must fit the open gripper
SHELF_FRONTAL_MAX_HALF_DEPTH = 0.05
SHELF_FRONTAL_PAST_CENTER = 0.03  # the finger pads sit behind the fingertip point
# Tilt down from horizontal, in order; level alone cannot reach a 0.39 m board (sim IK).
SHELF_FRONTAL_PITCHES = (10.0, 0.0, 20.0)
# Steeper tilts for a level with nothing above it: a 1.30 m top shelf is only reachable so (sim).
SHELF_FRONTAL_OPEN_TOP_PITCHES = (30.0, 45.0)
# Leaving the compartment: a straight MoveIt path this far up and back out.
SHELF_EXIT_LIFT = 0.02
SHELF_EXIT_RETREAT = 0.15
SHELF_EXIT_EDGE_MARGIN = (
    0.05  # with exit_to_edge: fingertips this far outside the board front
)
SHELF_EXIT_RETREAT_LIMITS = (0.05, 0.35)
SHELF_EXIT_VELOCITY = 0.3
SHELF_RETURN_VELOCITY = 0.75

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
    use_gpd: bool = False
    gpd_fallback: bool = True
    in_configuration: bool = False
    # False leaves the arm where the grasp ended, for callers that move on from
    # there themselves (the pour lifts straight up from the grasp).
    return_to_carry: bool = True
    # Back out until the fingertips clear the scanned board front instead of a fixed 15 cm.
    exit_to_edge: bool = False


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
    grasps: Optional[object] = None
    # Shelf compartment the cluster was cropped to, (min_z, max_z); None off a shelf.
    band: Optional[Tuple[float, float]] = None

    @property
    def fitted(self):
        """The generator's fitted collision object, if any."""
        grasps = self.grasps
        return grasps.object if grasps is not None and grasps.object.type else None


def execute(
    arm, perception, request: PickRequest, strategies
) -> Tuple[bool, PickOutcome]:
    """Pick an object. Returns (success, outcome)."""
    strategy_key = resolve_pick_strategy(
        request.object_name, force_gpd=request.is_shelf or request.use_gpd
    )
    return _pick(arm, perception, request, strategies, strategy_key)


def _pick(
    arm, perception, request: PickRequest, strategies, strategy_key: str
) -> Tuple[bool, PickOutcome]:
    log = arm.logger
    strategy = strategies[strategy_key]
    arm.set_context(strategy_key)

    log.info(
        f"[{strategy_key}] picking '{request.object_name}' (shelf={request.is_shelf})"
    )

    if not request.in_configuration:
        _move_to_stare_pose(arm, strategy_key)

    perceived = _perceive(arm, perception, request, strategy_key)
    if perceived is None:
        return _fall_back_or_fail(arm, perception, request, strategies, strategy)

    # Open after perceiving: the fingers are in the camera's view while it looks.
    # No settle: the strategy opens again before moving.
    arm.open_gripper(settle_s=0.0)

    fitted = perceived.fitted
    if fitted is not None:
        arm.add_collision_object(fitted)

    # Remember the perceived objects so a strategy can attach one.
    arm.snapshot_scene()

    outcome = None
    frontal = _shelf_frontal_set(arm, perceived) if request.is_shelf else None
    if frontal is not None:
        # Pre-grasp, straight in, close: the geometric picks' cartesian strategy.
        outcome = _run_strategy(arm, strategies[PICK_STRATEGY_GEOMETRIC], frontal)
    if outcome is None:
        for grasp_set in _grasp_sets(perception, perceived, request):
            outcome = _run_strategy(arm, strategy, grasp_set)
            if outcome is not None:
                break
            time.sleep(0.2)

    if outcome is None:
        log.error(f"[{strategy_key}] pick failed: no candidate succeeded")
        return _fall_back_or_fail(
            arm, perception, request, strategies, strategy, fitted
        )

    # Defensive re-close: the strategy already closed, but a missed service
    # response there would otherwise leave the object un-gripped for the return.
    arm.close_gripper(settle_s=0.0)

    outcome.object_name = request.object_name
    if fitted is not None:
        outcome.object_pick_height, outcome.object_height = _fitted_heights(
            fitted, outcome.pick_pose
        )
        if fitted.type == "cylinder" and is_tilted(outcome.pick_pose):
            outcome.place_top_down = True
            outcome.object_pick_height = _top_down_pick_height(outcome.object_height)
    if request.is_shelf:
        _shelf_exit(arm, request.exit_to_edge)
    if request.return_to_carry:
        _return_to_carry_pose(arm, strategy_key, request.is_shelf)
    log.info(f"[{strategy_key}] pick complete")
    return True, outcome


def _fall_back_or_fail(
    arm, perception, request: PickRequest, strategies, strategy, fitted=None
) -> Tuple[bool, PickOutcome]:
    if not (strategy.profile.fallback_to_gpd and request.gpd_fallback):
        return False, PickOutcome()
    arm.logger.warn(
        f"'{request.object_name}': {strategy.name} pick failed, falling back to GPD"
    )
    if fitted is not None:
        arm.remove_collision_object(fitted.id)
    return _pick(
        arm,
        perception,
        replace(request, in_configuration=False),
        strategies,
        PICK_STRATEGY_GPD,
    )


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
    arm, perception, request: PickRequest, strategy_key: str
) -> Optional[Perceived]:
    """Locate the object. Returns None when it cannot be found."""
    if strategy_key != PICK_STRATEGY_GPD:
        return _perceive_grasps(perception, request, strategy_key)
    return _perceive_cluster(arm, perception, request)


def _perceive_grasps(perception, request: PickRequest, strategy_key: str):
    """Ask the grasp generator for the object's grasp candidates.

    Deliberately does NOT cluster: clustering adds the table as a collision
    object, which makes MoveIt reject every near-table path.
    """
    perception.logger.info(
        f"'{request.object_name}': asking the grasp generator for candidates"
    )
    response = perception.generate_grasps(request.object_name)
    if response is None:
        return None

    # Rim/peak/bowl poses are used as-is (the strategy applies its own offset);
    # flat objects get the table-tuned tweak.
    z_tweak = FLAT_GRASP_Z_TWEAK if strategy_key == PICK_STRATEGY_FLAT else 0.0
    pose = response.pose
    pose.pose.position.z += z_tweak
    perception.logger.info(
        f"Generated grasp pose received ({response.samples_collected} samples), "
        f"z tweak={z_tweak}"
    )
    return Perceived(grasps=response)


def _perceive_cluster(arm, perception, request: PickRequest) -> Optional[Perceived]:
    """Detect the object and segment it into a cloud."""
    point = _locate(perception, request)
    if point is None:
        return None

    band = _shelf_band(arm, point) if request.is_shelf else None
    cluster = None
    for _ in range(3):
        cluster = perception.cluster_at(point, band=band)
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
    return Perceived(cluster=cluster, height=height, band=band)


def _shelf_band(arm, point: PointStamped) -> Optional[Tuple[float, float]]:
    """Z band of the object's compartment; the default -/+0.3 m band merges it with the boards."""
    levels = arm.shelf_levels()
    if not levels:
        arm.logger.warn("No shelf scan: clustering with the default height band")
        return None
    if point.header.frame_id == "base_link":
        z = point.point.z
    else:
        ok, in_base = transform_point(point, "base_link", arm.tf_buffer)
        if not ok:
            return None
        z = in_base.point.z
    band = compartment_band(z, levels)
    if band is not None:
        top = f"{band[1]:.3f} m" if band[1] else "the default band"
        arm.logger.info(f"Shelf compartment for clustering: z {band[0]:.3f} m to {top}")
    return band


def _shelf_frontal_set(arm, perceived: Perceived) -> Optional[GraspSet]:
    """Frontal grasps for an object isolated in its shelf compartment, or None."""
    if perceived.cluster is None or perceived.band is None:
        return None
    cloud = perceived.cluster
    points = _cloud_points(cloud)
    poses = shelf_frontal_grasps(points, perceived.band, cloud.header.frame_id)
    if not poses:
        size = np.ptp(points, axis=0) if len(points) else np.zeros(3)
        arm.logger.info(
            f"Shelf frontal grasp: a {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m "
            f"cluster ({len(points)} points) does not fit; using GPD"
        )
        return None
    # The cartesian approach drives the xArm TCP, whose frame is link_base.
    if cloud.header.frame_id != "link_base":
        moved = [transform_pose(pose, "link_base", arm.tf_buffer) for pose in poses]
        if not all(ok for ok, _ in moved):
            arm.logger.warn("Shelf frontal grasp: no TF to link_base; using GPD")
            return None
        poses = [pose for _, pose in moved]
    first = poses[0].pose.position
    arm.logger.info(
        f"Shelf frontal grasp: {len(poses)} height(s), first tip at "
        f"({first.x:.3f}, {first.y:.3f}, {first.z:.3f}) {poses[0].header.frame_id}"
    )
    return GraspSet(
        poses=poses, scores=[1.0 - 0.1 * i for i in range(len(poses))], source="frontal"
    )


def shelf_frontal_grasps(points, band, frame_id: str) -> List[PoseStamped]:
    """Frontal grasps from a z-up cluster in compartment ``band`` (max_z 0: nothing above).

    Tip just past the object's middle at mid height, approach +x tilted down, closing
    across y, camera up; ordered by height then pitch, empty when the object does not fit.
    """
    if points is None or len(points) < 10 or band is None:
        return []
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    width = float(np.percentile(y, 95) - np.percentile(y, 5))
    if width > GRIPPER_MAX_APERTURE - SHELF_FRONTAL_WIDTH_MARGIN:
        return []
    front = float(np.percentile(x, 5))
    half_depth = float(
        np.clip((np.percentile(x, 95) - front) / 2, 0.015, SHELF_FRONTAL_MAX_HALF_DEPTH)
    )
    centre_y = float(np.median(y))
    board = band[0] - SHELF_BAND_MARGIN
    top = float(np.percentile(z, 95))
    lowest = board + SHELF_FRONTAL_BOARD_CLEARANCE
    highest = top - SHELF_FRONTAL_TOP_MARGIN
    if band[1]:
        highest = min(
            highest, band[1] + SHELF_BAND_MARGIN - SHELF_FRONTAL_CEILING_CLEARANCE
        )
    if highest < lowest:
        return []
    middle = (board + top) / 2
    heights = []
    for step in SHELF_FRONTAL_Z_STEPS:
        h = float(np.clip(middle + step, lowest, highest))
        if all(abs(h - other) > 0.01 for other in heights):
            heights.append(h)

    pitches = SHELF_FRONTAL_PITCHES
    if not band[1]:  # nothing above: steeper approaches cannot hit a ceiling
        pitches = pitches + SHELF_FRONTAL_OPEN_TOP_PITCHES
    poses = []
    for h in heights:
        for pitch in pitches:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = front + half_depth + SHELF_FRONTAL_PAST_CENTER
            pose.pose.position.y = centre_y
            pose.pose.position.z = h
            poses.append(
                set_quaternion(
                    pose, R.from_euler("y", 90 + pitch, degrees=True).as_quat()
                )
            )
    _lift_camera_clear(poses)
    return poses


def compartment_band(z: float, levels) -> Optional[Tuple[float, float]]:
    """(min_z, max_z) of the compartment at ``z`` from (height, thickness) boards; 0 on top."""
    ordered = sorted(levels)
    below = [lv for lv in ordered if lv[0] - SHELF_BAND_DOWN_TOLERANCE <= z]
    if not below:
        return None
    floor = below[-1]
    above = [lv for lv in ordered if lv[0] > floor[0]]
    ceiling = (above[0][0] - above[0][1] - SHELF_BAND_MARGIN) if above else 0.0
    return floor[0] + SHELF_BAND_MARGIN, ceiling


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


def _cloud_points(cloud) -> np.ndarray:
    """(N, 3) x, y, z of a PointCloud2, NaNs dropped."""
    points = point_cloud2.read_points(
        cloud, field_names=("x", "y", "z"), skip_nans=True
    )
    return np.array([list(p) for p in points]).reshape(-1, 3)


def _cluster_height(cluster) -> Optional[float]:
    z = _cloud_points(cluster)[:, 2]
    return float(np.max(z) - np.min(z)) if len(z) else None


# ======================================================================
# Find grasp candidates
# ======================================================================


def _grasp_sets(
    perception, perceived: Perceived, request: PickRequest
) -> Iterator[GraspSet]:
    """Yield batches of candidates, best source first.

    Shapes yield the generator's best candidates. Flat objects yield the generator's
    one pose and its flip. GPD yields one set per config, so a config whose grasps
    are all unreachable falls back to the next.
    """
    response = perceived.grasps
    if response is not None:
        if perceived.fitted is not None:
            # Already ranked across every approach, flips included.
            poses, scores = list(response.grasps[:5]), list(response.scores[:5])
        else:
            # A 90 deg alternative would align the fingers with a fork's long axis
            # and collide with it, so the only alternative is the symmetric flip.
            poses = [response.pose, _rotate_about_approach(response.pose, 180)]
            scores = [1.0, 0.9]
        yield GraspSet(poses=poses, scores=scores, source="grasp_generator")
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


def _fitted_heights(fitted, grasp_pose) -> Tuple[float, float]:
    """The place pipeline's (pick height, object height) for the fitted object."""
    height = 2 * fitted.dimensions.x if fitted.type == "sphere" else fitted.dimensions.z
    bottom = fitted.pose.pose.position.z - height / 2
    return grasp_pose.pose.position.z - bottom + SAFETY_HEIGHT_GEOMETRIC, height


def _top_down_pick_height(object_height: float) -> float:
    """The pick height of the top-down grasp, which a tilted pick places with."""
    return object_height - TOP_GRASP_DEPTH + GRIPPER_REACH + SAFETY_HEIGHT_GEOMETRIC


# ======================================================================
# Return to a carry pose
# ======================================================================


def _shelf_exit(arm, to_edge: bool = False) -> bool:
    """Lift off the board, then back out along the way in, as straight MoveIt paths."""
    target = arm.link_pose(EEF_LINK_NAME)  # straight-line plans always move link_eef
    if target is None:
        return False
    back = -approach_axis(target)
    back[2] = 0.0
    norm = float(np.linalg.norm(back))
    steps = [np.array([0.0, 0.0, SHELF_EXIT_LIFT])]
    if norm >= 0.5:  # a steep grasp has no meaningful "straight back"
        retreat = (_retreat_to_edge(arm) if to_edge else None) or SHELF_EXIT_RETREAT
        steps.append(back / norm * retreat)
    with arm.phase("shelf_exit"):
        for step in steps:
            target = copy.deepcopy(target)
            target.pose.position.x += float(step[0])
            target.pose.position.y += float(step[1])
            target.pose.position.z += float(step[2])
            if not arm.move_to_pose(
                target,
                velocity=SHELF_EXIT_VELOCITY,
                planner_id=CARTESIAN_PLANNER,
                target_link=EEF_LINK_NAME,
            ):
                arm.logger.warn(
                    "Shelf exit: no straight path; the return plans around the boxes"
                )
                return False
    return True


def _retreat_to_edge(arm) -> Optional[float]:
    """Retreat that puts the fingertips SHELF_EXIT_EDGE_MARGIN outside the board front.

    In base_link, where the scan is; None without a scan or a backward approach.
    """
    extent = arm.shelf_extent()
    eef = arm.link_pose(EEF_LINK_NAME, "base_link")
    if extent is None or eef is None:
        return None
    approach = approach_axis(eef)
    back = -approach[:2] / max(float(np.linalg.norm(approach[:2])), 1e-6)
    if back[0] > -0.5:  # not backing out toward the robot
        return None
    p = eef.pose.position
    tip_x = p.x + approach[0] * GRIPPER_REACH
    retreat = (tip_x - extent[0] + SHELF_EXIT_EDGE_MARGIN) / -back[0]
    retreat = float(np.clip(retreat, *SHELF_EXIT_RETREAT_LIMITS))
    arm.logger.info(f"Shelf exit: {retreat * 100:.0f} cm back to clear the board front")
    return retreat


def _return_to_carry_pose(arm, strategy_key: str, is_shelf: bool) -> None:
    """Where the arm goes once it is holding the object."""
    log = arm.logger
    if strategy_key == PICK_STRATEGY_RIM:
        # Hold where the strategy left the arm; a basket must be carried in place.
        log.info("Rim pick: holding position (skipping return to stare)")
        return

    named = "look_side_stare" if strategy_key == PICK_STRATEGY_PEAK else "table_stare"
    # The shelf boxes are still in the scene, so one planned move leaves the shelf.
    velocity = SHELF_RETURN_VELOCITY if is_shelf else 0.5
    log.info(f"Returning to {named}")
    with arm.phase(f"return/{named}"):
        for _ in range(5):
            arm.clear_octomap()
            if arm.move_to_named_position(named, velocity=velocity):
                return
        log.warn(f"Could not return to {named} after 5 attempts")


__all__ = ["GraspSet", "PickOutcome", "PickRequest", "Perceived", "execute"]
