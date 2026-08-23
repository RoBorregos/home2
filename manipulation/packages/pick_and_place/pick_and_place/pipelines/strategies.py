"""Pick motion strategies: how the arm actually closes on an object.

Three algorithms cover every object the robot picks. Which one runs is decided
by ``resolve_pick_strategy(object_name)`` and looked up in ``STRATEGIES`` at the
bottom of this file; the numbers each one uses come from
``config/pick_profiles.yaml``.
"""

from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import PoseStamped

from pick_and_place.pipelines.classification import (
    PICK_STRATEGY_BOWL,
    PICK_STRATEGY_FLAT,
    PICK_STRATEGY_GPD,
    PICK_STRATEGY_PEAK,
    PICK_STRATEGY_RIM,
)
from pick_and_place.pipelines.errors import PickAttemptFailed
from pick_and_place.pipelines.profiles import PickProfile
from pick_and_place.robot.geometry import offset_z


@dataclass(frozen=True)
class GraspCandidate:
    """One grasp attempt: a pose in the EE-link frame plus its provenance.

    The tip offset and the per-alternative step are already applied to ``pose``
    by the pick pipeline, so strategies work in EE-link coordinates throughout.
    """

    pose: PoseStamped
    score: float
    pose_index: int
    alternative_index: int

    def label(self) -> str:
        return f"pose={self.pose_index} alt={self.alternative_index}"


@dataclass
class PickOutcome:
    """What a successful pick achieved.

    Carried from the pick pipeline to the place pipeline. The height fields stay
    zero for strategies that attach no collision object, and the place pipeline
    treats zero as "unknown".
    """

    pick_pose: Optional[PoseStamped] = None
    grasp_score: float = 0.0
    object_name: str = ""
    object_pick_height: float = 0.0
    object_height: float = 0.0


class PickStrategy:
    """Base for the three motion strategies."""

    def __init__(self, profile: PickProfile):
        self.profile = profile

    @property
    def name(self) -> str:
        return self.profile.name

    def attempt(self, arm, candidate: GraspCandidate) -> PickOutcome:
        """Execute one grasp attempt.

        Returns a PickOutcome on success. Raises PickAttemptFailed to move on to
        the next candidate, or PickAborted / PickHardwareError to stop the task.
        """
        raise NotImplementedError


class FixedDistanceDescentPick(PickStrategy):
    """Rise to a pre-grasp above the target, then descend a known distance.

    Used where the target height is trustworthy: the rim of a basket or bowl, or
    the top of a pile of clothes. The descent runs under xArm cartesian velocity
    closed on TCP Z, so the octomap noise these cavities generate cannot affect
    it. Rim, bowl and peak differ only in their profile, the descent being
    ``pre_grasp_height - grasp_z_tweak``.
    """

    def attempt(self, arm, candidate: GraspCandidate) -> PickOutcome:
        profile = self.profile

        if profile.validate_endpoint:
            with arm.phase("validate_endpoint"):
                endpoint = offset_z(candidate.pose, profile.grasp_z_tweak)
                if arm.endpoint_self_collides(endpoint):
                    raise PickAttemptFailed("descent endpoint self-collides with robot")

        with arm.phase("open_gripper"):
            arm.open_gripper()

        pre_grasp = offset_z(candidate.pose, profile.pre_grasp_height)
        with arm.phase("approach_pre_grasp"):
            arm.clear_octomap()
            if not arm.move_to_pregrasp(pre_grasp, profile.pre_grasp_velocity):
                raise PickAttemptFailed("pre-grasp unreachable")

        with arm.phase("descend"):
            # Clear again: the arm has moved
            arm.clear_octomap()
            arm.check_abort()
            descent = profile.effective_descent_distance
            if not arm.fixed_distance_descent(descent, profile.descent_speed):
                raise PickAttemptFailed(
                    f"fixed-distance descent of {descent * 1000:.0f} mm did not complete"
                )

        with arm.phase("close_gripper"):
            arm.close_gripper(settle_s=profile.close_settle)

        # No lift: the pick pipeline chooses the return pose.
        return PickOutcome(pick_pose=candidate.pose, grasp_score=candidate.score)


class ForceGuardedDescentPick(PickStrategy):
    """Descend onto a flat object until joint effort reports contact.

    Forks, plates and sponges are too thin to descend onto by position: the
    perceived height is not accurate to the millimetre, so a fixed descent
    either stops short or drives into the table.

    The pre-grasp is a plain pose goal (top-down and unobstructed, so nearest-IK
    seeding buys nothing) and the arm lifts back to it afterwards.
    """

    def attempt(self, arm, candidate: GraspCandidate) -> PickOutcome:
        profile = self.profile

        with arm.phase("open_gripper"):
            arm.open_gripper()

        pre_grasp = offset_z(candidate.pose, profile.pre_grasp_height)
        with arm.phase("approach_pre_grasp"):
            arm.clear_octomap()
            if not arm.move_to_pose(pre_grasp, velocity=profile.pre_grasp_velocity):
                raise PickAttemptFailed("pre-grasp unreachable")

        with arm.phase("descend"):
            arm.clear_octomap()
            arm.check_abort()
            result = arm.force_guarded_descent(profile.force_guard)
            if not result.contact:
                raise PickAttemptFailed(
                    f"no contact after descending {result.descended_m * 1000:.0f} mm"
                )

        with arm.phase("retract"):
            # Retract from the measured contact point, never the nominal target.
            contact_z = pre_grasp.pose.position.z - result.descended_m
            retract_pose = offset_z(pre_grasp, 0.0)
            retract_pose.pose.position.z = contact_z + profile.post_contact_retract
            arm.move_to_pose(retract_pose, velocity=0.1)

        with arm.phase("close_gripper"):
            arm.close_gripper(settle_s=profile.close_settle)

        if profile.lift_after_grasp:
            with arm.phase("lift"):
                arm.move_to_pose(pre_grasp, velocity=0.2)

        # Heights stay zero: this strategy attaches no collision object.
        return PickOutcome(pick_pose=candidate.pose, grasp_score=candidate.score)


class DirectGraspPick(PickStrategy):
    """Plan straight to a GPD-generated grasp and close on it.

    The default for anything without a specialised behavior. GPD grasps already
    approach from a reachable direction, so MoveIt plans the whole approach in
    one go, with no pre-grasp or descent phase. It is also the only strategy
    that attaches the object in the planning scene, which is what lets it report
    the heights the place pipeline needs.
    """

    def attempt(self, arm, candidate: GraspCandidate) -> PickOutcome:
        with arm.phase("move_to_grasp"):
            if not arm.move_to_pose(candidate.pose):
                raise PickAttemptFailed("grasp pose unreachable")

        with arm.phase("attach_object"):
            attachment = arm.attach_pick_object()

        with arm.phase("close_gripper"):
            arm.close_gripper(settle_s=self.profile.close_settle)

        outcome = PickOutcome(pick_pose=candidate.pose, grasp_score=candidate.score)

        if not attachment.attached:
            arm.logger.error(
                "Failed to attach object to the planning scene; "
                "reporting the pick without object heights"
            )
            return outcome

        if attachment.lowest_object is not None:
            outcome.object_pick_height = arm.object_pick_height(
                attachment.lowest_object, candidate.pose
            )
        if (
            attachment.lowest_object is not None
            and attachment.highest_object is not None
        ):
            outcome.object_height = arm.object_height(
                attachment.lowest_object, attachment.highest_object
            )
        return outcome


# Which strategy handles which profile.
STRATEGIES = {
    PICK_STRATEGY_FLAT: ForceGuardedDescentPick,
    PICK_STRATEGY_RIM: FixedDistanceDescentPick,
    PICK_STRATEGY_BOWL: FixedDistanceDescentPick,
    PICK_STRATEGY_PEAK: FixedDistanceDescentPick,
    PICK_STRATEGY_GPD: DirectGraspPick,
}


def build_strategies(profiles):
    """Instantiate one strategy per loaded profile.

    Called once at node startup so a missing or mismatched profile fails loudly
    at launch rather than on the first pick of that type.
    """
    unclaimed = set(profiles) - set(STRATEGIES)
    if unclaimed:
        raise KeyError(
            f"No strategy registered for profile(s): {sorted(unclaimed)}. "
            f"Known: {sorted(STRATEGIES)}"
        )
    return {key: STRATEGIES[key](profile) for key, profile in profiles.items()}
