"""The place pipeline end to end.

Several of these pin behaviour that the four-nodes-to-one merge changed by
accident: the heatmap bias on tables, the MoveIt tolerances, and the service
timeouts. They exist so a future refactor cannot re-introduce them silently.
"""

import pytest
from frida_interfaces.msg import PlaceParams

from pick_and_place.pipelines import place as place_pipeline
from pick_and_place.pipelines.strategies import PickOutcome

from fakes import (
    FakeArm,
    FakeFlatResponse,
    FakePerception,
    make_cluster,
    make_point,
    make_pose,
)


def params(**overrides):
    """PlaceParams with the fields the pipeline reads."""
    p = PlaceParams()
    p.table_height = overrides.pop("table_height", 0.75)
    p.is_shelf = overrides.pop("is_shelf", False)
    p.is_trash = overrides.pop("is_trash", False)
    p.close_to = overrides.pop("close_to", "")
    p.special_request = overrides.pop("special_request", "")
    p.skip_initial_pose = overrides.pop("skip_initial_pose", False)
    if "forced_pose" in overrides:
        p.forced_pose = overrides.pop("forced_pose")
    assert not overrides, f"unknown PlaceParams fields: {sorted(overrides)}"
    return p


def perception(**overrides):
    overrides.setdefault("heatmap_point", make_point(z=0.70))
    overrides.setdefault("surface_cloud", make_cluster())
    return FakePerception(**overrides)


def held(height=0.12):
    """An outcome from a successful pick, as place receives it."""
    return PickOutcome(pick_pose=make_pose(z=0.5), object_pick_height=height)


def run(arm, perc, place_params, outcome=None):
    return place_pipeline.execute(arm, perc, place_params, outcome or held())


# --- heatmap bias (regression) ----------------------------------------------


def test_tables_do_not_bias_toward_the_nearest_point():
    """prefer_closest must stay unset on a table: the merge set it True."""
    arm, perc = FakeArm(), perception()
    run(arm, perc, params(is_shelf=False))
    request = perc.heatmap_place_client.requests[0]
    assert request.prefer_closest is False


@pytest.mark.parametrize(
    "table_height,expected",
    [(0.75, False), (1.20, True)],
)
def test_shelf_bias_depends_on_height(table_height, expected):
    """High shelves prefer the nearest reachable point; low ones the centre."""
    arm, perc = FakeArm(), perception()
    run(arm, perc, params(is_shelf=True, table_height=table_height))
    assert perc.heatmap_place_client.requests[0].prefer_closest is expected


# --- planning tolerances (regression) ---------------------------------------


def test_place_uses_the_planners_own_tolerances_not_picks():
    """place_server left these unset; pick's tighter defaults broke planning."""
    arm, perc = FakeArm(), perception()
    run(arm, perc, params())
    assert arm.move_kwargs, "the place must have attempted a motion"
    for kwargs in arm.move_kwargs:
        assert kwargs["tolerance_position"] == pytest.approx(0.01)
        assert kwargs["tolerance_orientation"] == pytest.approx(0.05)


# --- timeouts (regression) ---------------------------------------------------


def test_trash_detection_keeps_the_long_estimator_timeout():
    perc = perception(flat_response=FakeFlatResponse(make_pose(z=0.6)))
    run(FakeArm(), perc, params(is_trash=True))
    assert perc.flat_timeouts == [pytest.approx(15.0)]


# --- the height ladder -------------------------------------------------------


def test_table_ladder_walks_eight_rungs_of_five_centimetres():
    arm = FakeArm(move_to_pose_results=[False] * 8)
    run(arm, perception(), params(is_shelf=False))
    zs = [p.pose.position.z for p in arm.poses["move_to_pose"]]
    assert len(zs) == 8
    assert zs == pytest.approx([zs[0] + i * 0.05 for i in range(8)])


def test_shelf_ladder_walks_three_rungs_of_two_centimetres():
    """Fewer, finer rungs: the compartment ceiling is close."""
    arm = FakeArm(move_to_pose_results=[True, False, False] * 3)
    run(arm, perception(), params(is_shelf=True))
    assert arm.calls.count("move_to_pose") <= 3 * 3


def test_first_reachable_rung_wins():
    arm = FakeArm(move_to_pose_results=[False, True])
    assert run(arm, perception(), params(is_shelf=False)) is True
    assert arm.calls.count("move_to_pose") == 2


def test_place_fails_when_no_rung_is_reachable():
    arm = FakeArm(move_to_pose_results=[False] * 8)
    assert run(arm, perception(), params(is_shelf=False)) is False
    assert "detach_pick_objects" not in arm.calls


# --- release and return ------------------------------------------------------


def test_success_detaches_then_opens_the_gripper():
    arm = FakeArm()
    run(arm, perception(), params())
    assert arm.calls.index("detach_pick_objects") < arm.calls.index("open_gripper")


def test_shelf_guard_is_added_and_always_removed():
    arm = FakeArm()
    run(arm, perception(), params(is_shelf=True))
    assert "add_shelf_ceiling_guard" in arm.calls
    assert "remove_shelf_ceiling_guard" in arm.calls
    assert arm.calls.index("add_shelf_ceiling_guard") < arm.calls.index(
        "remove_shelf_ceiling_guard"
    )


def test_guard_is_removed_even_when_the_return_fails():
    arm = FakeArm(named_position_results=[False] * 10)
    run(arm, perception(), params(is_shelf=True))
    assert "remove_shelf_ceiling_guard" in arm.calls


def test_tables_use_no_ceiling_guard():
    arm = FakeArm()
    run(arm, perception(), params(is_shelf=False))
    assert "add_shelf_ceiling_guard" not in arm.calls


def test_initial_pose_is_skipped_when_asked():
    arm = FakeArm()
    run(arm, perception(), params(skip_initial_pose=True))
    # Only the return move remains.
    assert arm.named_positions == ["table_stare"]


# --- drop height -------------------------------------------------------------


def test_table_drop_lifts_by_the_measured_pick_height():
    arm = FakeArm()
    run(arm, perception(), params(is_shelf=False), outcome=held(height=0.12))
    # heatmap point z=0.70, plus the object's grasp height.
    assert arm.poses["move_to_pose"][0].pose.position.z == pytest.approx(0.82)


def test_shelf_drop_uses_a_fixed_clearance_not_the_object_height():
    """On a shelf the compartment ceiling sets the clearance, not the object."""
    arm = FakeArm()
    run(arm, perception(), params(is_shelf=True), outcome=held(height=0.12))
    # Index 1 is the final pose; index 0 is the pre-place approach.
    assert arm.poses["move_to_pose"][1].pose.position.z == pytest.approx(0.80)


def test_shelf_approaches_from_a_pre_place_pose_first():
    arm = FakeArm()
    run(arm, perception(), params(is_shelf=True))
    pre, final = arm.poses["move_to_pose"][0], arm.poses["move_to_pose"][1]
    assert (pre.pose.position.x, pre.pose.position.y) != (
        final.pose.position.x,
        final.pose.position.y,
    ), "the pre-place must stand off from the final pose"


def test_unknown_pick_height_falls_back_to_a_default():
    arm = FakeArm()
    run(arm, perception(), params(is_shelf=False), outcome=PickOutcome())
    assert arm.poses["move_to_pose"][0].pose.position.z == pytest.approx(0.80)


# --- close_to ----------------------------------------------------------------


def test_close_to_reaches_the_heatmap():
    perc = perception(located_point=make_point(x=0.4), cluster=make_cluster())
    run(FakeArm(), perc, params(close_to="cup"))
    assert perc.heatmap_place_client.requests[0].close_point is not None


def test_unresolvable_close_to_degrades_instead_of_failing():
    """The old place ignored close_to entirely; failing the task is worse."""
    perc = perception(located_point=None)
    assert run(FakeArm(), perc, params(close_to="ghost")) is True


# --- trash and forced pose ---------------------------------------------------


def test_trash_place_skips_the_heatmap_entirely():
    perc = perception(flat_response=FakeFlatResponse(make_pose(z=0.6)))
    assert run(FakeArm(), perc, params(is_trash=True)) is True
    assert not perc.heatmap_place_client.requests


def test_trash_place_fails_when_the_bin_is_not_found():
    perc = perception(flat_response=None)
    assert run(FakeArm(), perc, params(is_trash=True)) is False


def test_missing_surface_cloud_fails_before_moving():
    arm = FakeArm()
    assert run(arm, perception(surface_cloud=None), params()) is False
    assert "move_to_pose" not in arm.calls
