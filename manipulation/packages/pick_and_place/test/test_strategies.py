"""Per-strategy behavior, asserted against a fake arm.

A pick strategy is defined by the sequence of things it does to the robot, so
most of these tests assert on ``FakeArm.calls``.
"""

from pathlib import Path

import pytest

from pick_and_place.pipelines.classification import (
    PICK_STRATEGY_BOWL,
    PICK_STRATEGY_FLAT,
    PICK_STRATEGY_GPD,
    PICK_STRATEGY_PEAK,
    PICK_STRATEGY_RIM,
)
from pick_and_place.pipelines.errors import PickAborted, PickAttemptFailed
from pick_and_place.pipelines.profiles import load_profiles
from pick_and_place.pipelines.strategies import (
    DirectGraspPick,
    FixedDistanceDescentPick,
    ForceGuardedDescentPick,
    GraspCandidate,
    build_strategies,
)
from pick_and_place.robot.arm import AttachResult, ContactResult

from fakes import FakeArm, FakeObject, make_pose

PROFILES_PATH = Path(__file__).resolve().parents[1] / "config" / "pick_profiles.yaml"


@pytest.fixture
def strategies():
    return build_strategies(load_profiles(PROFILES_PATH))


def candidate(z=0.3, score=0.9):
    return GraspCandidate(
        pose=make_pose(z=z), score=score, pose_index=0, alternative_index=0
    )


# --- registry ------------------------------------------------------------


def test_registry_maps_every_profile_to_the_expected_class(strategies):
    assert isinstance(strategies[PICK_STRATEGY_RIM], FixedDistanceDescentPick)
    assert isinstance(strategies[PICK_STRATEGY_BOWL], FixedDistanceDescentPick)
    assert isinstance(strategies[PICK_STRATEGY_PEAK], FixedDistanceDescentPick)
    assert isinstance(strategies[PICK_STRATEGY_FLAT], ForceGuardedDescentPick)
    assert isinstance(strategies[PICK_STRATEGY_GPD], DirectGraspPick)


def test_rim_bowl_and_peak_share_one_class_with_different_profiles(strategies):
    """The collapse this refactor is built on: one algorithm, three profiles."""
    assert (
        type(strategies["rim"]) is type(strategies["bowl"]) is type(strategies["peak"])
    )
    descents = {
        name: strategies[name].profile.effective_descent_distance
        for name in ("rim", "bowl", "peak")
    }
    assert descents == pytest.approx({"rim": 0.15, "bowl": 0.08, "peak": 0.15})


def test_build_rejects_a_profile_with_no_registered_class():
    profiles = load_profiles(PROFILES_PATH)
    profiles["invented"] = profiles["gpd"]
    with pytest.raises(KeyError, match="No strategy registered"):
        build_strategies(profiles)


# --- fixed-distance strategy ---------------------------------------------


def test_rim_call_sequence(strategies):
    arm = FakeArm()
    strategies["rim"].attempt(arm, candidate())
    assert arm.calls == [
        "endpoint_self_collides",
        "open_gripper",
        "clear_octomap",
        "move_to_pregrasp",
        "clear_octomap",
        "fixed_distance_descent",
        "close_gripper",
    ]


@pytest.mark.parametrize(
    "name,expected_distance", [("rim", 0.15), ("bowl", 0.08), ("peak", 0.15)]
)
def test_fixed_distance_descends_the_profile_distance(
    strategies, name, expected_distance
):
    arm = FakeArm()
    strategies[name].attempt(arm, candidate())
    distance, speed = arm.descents[0]
    assert distance == pytest.approx(expected_distance)
    assert speed == pytest.approx(20.0)


def test_rim_validates_the_endpoint_at_the_z_tweak(strategies):
    arm = FakeArm()
    strategies["rim"].attempt(arm, candidate(z=0.3))
    validated = arm.poses["endpoint_self_collides"][0]
    # RIM_GRASP_Z_TWEAK is -0.05, so the endpoint sits below the grasp.
    assert validated.pose.position.z == pytest.approx(0.25)


def test_pre_grasp_is_raised_by_the_profile_height(strategies):
    arm = FakeArm()
    strategies["rim"].attempt(arm, candidate(z=0.3))
    pre_grasp = arm.poses["move_to_pregrasp"][0]
    assert pre_grasp.pose.position.z == pytest.approx(0.40)


def test_self_collision_skips_the_candidate_before_touching_the_robot(strategies):
    arm = FakeArm(self_collides=[True])
    with pytest.raises(PickAttemptFailed, match="self-collides"):
        strategies["rim"].attempt(arm, candidate())
    assert arm.calls == ["endpoint_self_collides"]


def test_unreachable_pre_grasp_fails_the_attempt(strategies):
    arm = FakeArm(pregrasp_results=[False])
    with pytest.raises(PickAttemptFailed, match="pre-grasp unreachable"):
        strategies["rim"].attempt(arm, candidate())
    assert "fixed_distance_descent" not in arm.calls


def test_incomplete_descent_fails_the_attempt(strategies):
    arm = FakeArm(descent_results=[False])
    with pytest.raises(PickAttemptFailed, match="did not complete"):
        strategies["rim"].attempt(arm, candidate())
    assert "close_gripper" not in arm.calls


def test_estop_during_descent_aborts_rather_than_retrying(strategies):
    arm = FakeArm(abort_after=0)
    with pytest.raises(PickAborted):
        strategies["rim"].attempt(arm, candidate())


def test_fixed_distance_populates_the_pick_result(strategies):
    """Regression: rim and peak used to return an empty PickResult."""
    arm = FakeArm()
    outcome = strategies["rim"].attempt(arm, candidate(score=0.77))
    assert outcome.pick_pose is not None
    assert outcome.grasp_score == pytest.approx(0.77)


def test_fixed_distance_never_lifts(strategies):
    """The pick pipeline chooses the return pose; baskets are held in place."""
    arm = FakeArm()
    strategies["rim"].attempt(arm, candidate())
    assert "move_to_pose" not in arm.calls


# --- force-guarded strategy ----------------------------------------------


def test_flat_call_sequence(strategies):
    arm = FakeArm()
    strategies["flat"].attempt(arm, candidate())
    assert arm.calls == [
        "open_gripper",
        "clear_octomap",
        "move_to_pose",  # pre-grasp
        "clear_octomap",
        "force_guarded_descent",
        "move_to_pose",  # retract
        "close_gripper",
        "move_to_pose",  # lift
    ]


def test_flat_skips_the_self_collision_check(strategies):
    arm = FakeArm()
    strategies["flat"].attempt(arm, candidate())
    assert "endpoint_self_collides" not in arm.calls


def test_flat_retracts_from_the_measured_contact_point(strategies):
    """The retract must be computed from how far the arm actually descended.

    Using the nominal target instead would drive the tool back down into the
    table whenever contact happened early.
    """
    arm = FakeArm(contact_results=[ContactResult(contact=True, descended_m=0.11)])
    strategies["flat"].attempt(arm, candidate(z=0.30))
    retract = arm.poses["move_to_pose"][1]
    # pre-grasp 0.30 + 0.15 = 0.45; contact at 0.45 - 0.11 = 0.34; +0.004 retract.
    assert retract.pose.position.z == pytest.approx(0.344)


def test_flat_retract_is_always_above_the_contact_point(strategies):
    for descended in (0.09, 0.12, 0.15):
        arm = FakeArm(
            contact_results=[ContactResult(contact=True, descended_m=descended)]
        )
        strategies["flat"].attempt(arm, candidate(z=0.30))
        contact_z = 0.45 - descended
        assert arm.poses["move_to_pose"][1].pose.position.z > contact_z


def test_flat_without_contact_fails_the_attempt(strategies):
    arm = FakeArm(contact_results=[ContactResult(contact=False, descended_m=0.15)])
    with pytest.raises(PickAttemptFailed, match="no contact"):
        strategies["flat"].attempt(arm, candidate())
    assert "close_gripper" not in arm.calls


def test_flat_lifts_back_to_the_pre_grasp(strategies):
    arm = FakeArm()
    strategies["flat"].attempt(arm, candidate(z=0.30))
    pre_grasp, lift = arm.poses["move_to_pose"][0], arm.poses["move_to_pose"][2]
    assert lift.pose.position.z == pytest.approx(pre_grasp.pose.position.z)


def test_flat_uses_the_longer_close_settle(strategies):
    arm = FakeArm()
    strategies["flat"].attempt(arm, candidate())
    assert arm.close_settles == [2.5]


def test_flat_passes_its_force_guard_profile_through(strategies):
    arm = FakeArm()
    strategies["flat"].attempt(arm, candidate())
    guard = arm.guards[0]
    assert guard.jump_trip == 2.5
    assert guard.ignore_joints == (0, 5)


# --- direct grasp strategy ------------------------------------------------


def test_gpd_call_sequence(strategies):
    arm = FakeArm()
    strategies["gpd"].attempt(arm, candidate())
    assert arm.calls[:3] == ["move_to_pose", "attach_pick_object", "close_gripper"]


def test_gpd_reports_object_heights(strategies):
    arm = FakeArm(
        attach_result=AttachResult(
            attached=True, lowest_object=FakeObject(), highest_object=FakeObject()
        )
    )
    outcome = strategies["gpd"].attempt(arm, candidate())
    assert outcome.object_pick_height == pytest.approx(0.05)
    assert outcome.object_height == pytest.approx(0.12)


def test_gpd_unreachable_grasp_fails_the_attempt(strategies):
    arm = FakeArm(move_to_pose_results=[False])
    with pytest.raises(PickAttemptFailed, match="unreachable"):
        strategies["gpd"].attempt(arm, candidate())
    assert "attach_pick_object" not in arm.calls


def test_gpd_still_reports_the_pick_when_attaching_fails(strategies):
    """The grasp succeeded; only the scene bookkeeping did not."""
    arm = FakeArm(attach_result=AttachResult(attached=False))
    outcome = strategies["gpd"].attempt(arm, candidate(score=0.6))
    assert outcome.grasp_score == pytest.approx(0.6)
    assert outcome.object_pick_height == 0.0
    assert arm.logger.error_messages


def test_gpd_handles_a_missing_lowest_object(strategies):
    arm = FakeArm(
        attach_result=AttachResult(
            attached=True, lowest_object=None, highest_object=None
        )
    )
    outcome = strategies["gpd"].attempt(arm, candidate())
    assert outcome.object_pick_height == 0.0
    assert outcome.object_height == 0.0
