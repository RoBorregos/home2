"""The pick pipeline end to end.

These tests were impossible before the merge: a pick spanned two processes and
a ROS action, so nothing could drive one without a live graph. Now it is a
single callable over two fakes.
"""

from pathlib import Path

import pytest

from pick_and_place.pipelines import pick as pick_pipeline
from pick_and_place.pipelines.classification import (
    PICK_STRATEGY_GPD,
    resolve_pick_strategy,
)
from pick_and_place.pipelines.errors import PickAborted
from pick_and_place.pipelines.profiles import load_profiles
from pick_and_place.pipelines.strategies import build_strategies
from pick_and_place.robot.arm import ContactResult

from fakes import (
    FakeArm,
    FakeFlatResponse,
    FakePerception,
    make_cluster,
    make_point,
    make_pose,
)

PROFILES_PATH = Path(__file__).resolve().parents[1] / "config" / "pick_profiles.yaml"


@pytest.fixture
def strategies():
    return build_strategies(load_profiles(PROFILES_PATH))


def flat_perception(z=0.5):
    return FakePerception(flat_response=FakeFlatResponse(make_pose(z=z)))


def gpd_perception(n_grasps=2):
    return FakePerception(
        located_point=make_point(),
        cluster=make_cluster(),
        grasps=[([make_pose()] * n_grasps, [0.9] * n_grasps)],
    )


def run(arm, perception, strategies, **kwargs):
    request = pick_pipeline.PickRequest(**kwargs)
    return pick_pipeline.execute(arm, perception, request, strategies)


# --- stare poses -----------------------------------------------------------


@pytest.mark.parametrize(
    "object_name,expected",
    [
        # Every flat object shares one pose -- utensils and larger flat items alike.
        ("fork", "flat_stare"),
        ("knife", "flat_stare"),
        ("spoon", "flat_stare"),
        ("plate", "flat_stare"),
        ("red_plate", "flat_stare"),
        ("toothpaste", "flat_stare"),
        ("sponge", "flat_stare"),
        ("dishwasher_tab", "flat_stare"),
        ("basket", "look_side_stare"),
        ("clothes", "look_side_stare"),
        ("bowl", "table_stare"),
        ("cup", "table_stare"),  # GPD default
    ],
)
def test_stare_pose_per_object(strategies, object_name, expected):
    arm = FakeArm()
    # Only GPD objects go through clustering; everything else uses the estimator.
    is_gpd = resolve_pick_strategy(object_name) == PICK_STRATEGY_GPD
    perception = gpd_perception() if is_gpd else flat_perception()
    run(arm, perception, strategies, object_name=object_name)
    assert arm.named_positions[0] == expected


def test_in_configuration_skips_the_stare(strategies):
    arm = FakeArm()
    run(arm, flat_perception(), strategies, object_name="fork", in_configuration=True)
    assert "flat_stare" not in arm.named_positions


# --- ordering --------------------------------------------------------------


def test_gripper_opens_after_perceiving_and_before_grasping(strategies):
    """The fingers are in the camera's view, so the open must follow the look."""
    arm = FakeArm()
    perception = flat_perception()
    run(arm, perception, strategies, object_name="fork")

    open_index = arm.calls.index("open_gripper")
    descend_index = arm.calls.index("force_guarded_descent")
    assert perception.calls[0] == "estimate_flat_grasp"
    assert open_index < descend_index


def test_scene_is_snapshotted_before_the_strategy_runs(strategies):
    arm = FakeArm()
    run(arm, gpd_perception(), strategies, object_name="cup")
    assert arm.calls.index("snapshot_scene") < arm.calls.index("attach_pick_object")


# --- return poses ----------------------------------------------------------


def test_rim_holds_position(strategies):
    arm = FakeArm()
    run(arm, flat_perception(), strategies, object_name="basket")
    # Only the initial stare; no return move.
    assert arm.named_positions == ["look_side_stare"]


def test_peak_returns_to_look_side_stare(strategies):
    arm = FakeArm()
    run(arm, flat_perception(), strategies, object_name="clothes")
    assert arm.named_positions[-1] == "look_side_stare"


def test_bowl_returns_to_table_stare(strategies):
    arm = FakeArm()
    run(arm, flat_perception(), strategies, object_name="bowl")
    assert arm.named_positions[-1] == "table_stare"


def test_shelf_pick_retracts_to_front_stare_then_returns(strategies):
    """front_stare clears the compartment ceiling; the normal return still runs."""
    arm = FakeArm()
    run(arm, gpd_perception(), strategies, object_name="cup", is_shelf=True)
    assert "front_stare" in arm.named_positions
    assert arm.named_positions[-1] == "table_stare"


# --- failures --------------------------------------------------------------


def test_missing_flat_pose_fails_without_moving(strategies):
    arm = FakeArm()
    success, outcome = run(
        arm, FakePerception(flat_response=None), strategies, object_name="fork"
    )
    assert not success
    assert "force_guarded_descent" not in arm.calls


def test_object_not_found_fails(strategies):
    arm = FakeArm()
    perception = FakePerception(located_point=None)
    success, _ = run(arm, perception, strategies, object_name="cup")
    assert not success


def test_gpd_falls_back_to_the_next_config(strategies):
    """First config yields nothing; the second must still be tried."""
    perception = FakePerception(
        located_point=make_point(),
        cluster=make_cluster(),
        grasps=[([], []), ([make_pose()], [0.9])],
    )
    arm = FakeArm()
    success, _ = run(arm, perception, strategies, object_name="cup")
    assert success
    assert perception.calls.count("detect_grasps") == 2


def test_abort_propagates_out_of_the_pipeline(strategies):
    arm = FakeArm(abort_after=0)
    with pytest.raises(PickAborted):
        run(arm, flat_perception(), strategies, object_name="basket")


def test_every_candidate_failing_reports_failure(strategies):
    arm = FakeArm(descent_results=[False] * 10)
    success, outcome = run(arm, flat_perception(), strategies, object_name="basket")
    assert not success
    assert outcome.pick_pose is None


# --- outcome ---------------------------------------------------------------


@pytest.mark.parametrize("object_name", ["fork", "basket", "bowl", "clothes"])
def test_flat_strategies_populate_the_outcome(strategies, object_name):
    """Regression: rim/bowl/peak used to return an empty result, so the place
    pipeline got zeroed heights."""
    arm = FakeArm()
    success, outcome = run(arm, flat_perception(), strategies, object_name=object_name)
    assert success
    assert outcome.object_name == object_name
    assert outcome.pick_pose is not None
    assert outcome.grasp_score == pytest.approx(1.0)


def test_gpd_reports_object_heights(strategies):
    arm = FakeArm()
    success, outcome = run(arm, gpd_perception(), strategies, object_name="cup")
    assert success
    assert outcome.object_pick_height == pytest.approx(0.05)
    assert outcome.object_height == pytest.approx(0.12)


# --- geometry --------------------------------------------------------------


def test_tip_offset_uses_the_per_strategy_parameter(strategies):
    """rim_tip_offset is -0.18, and the rim pre-grasp sits 0.10 above."""
    arm = FakeArm()
    run(arm, flat_perception(z=0.50), strategies, object_name="basket")
    assert arm.poses["move_to_pregrasp"][0].pose.position.z == pytest.approx(0.42)


def test_flat_alternatives_step_the_offset(strategies):
    arm = FakeArm(contact_results=[ContactResult(contact=False, descended_m=0.15)] * 12)
    run(arm, flat_perception(z=0.50), strategies, object_name="fork")
    # Estimator pose 0.50, +FLAT_GRASP_Z_TWEAK 0.076 (flat strategy only),
    # ee_link_offset -0.09, pre-grasp +0.15 => 0.636, then -0.005 per alternative.
    base = 0.50 + pick_pipeline.FLAT_GRASP_Z_TWEAK - 0.09 + 0.15
    expected = [base + i * -0.005 for i in range(6)]
    zs = [p.pose.position.z for p in arm.poses["move_to_pose"]]
    assert zs[:6] == pytest.approx(expected)
    # Two poses (the estimator pose and its 180 deg twin) x six alternatives.
    assert len(zs) == 12


def test_only_flat_gets_the_flat_z_tweak(strategies):
    """Rim/bowl/peak use the estimator pose as-is; the strategy offsets it."""
    arm = FakeArm()
    run(arm, flat_perception(z=0.50), strategies, object_name="basket")
    # rim_tip_offset -0.18, pre-grasp +0.10, no tweak.
    assert arm.poses["move_to_pregrasp"][0].pose.position.z == pytest.approx(0.42)
