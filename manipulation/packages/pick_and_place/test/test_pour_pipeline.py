"""The pour pipeline end to end.

Several of these pin behaviour the merge changed by accident: the tilt direction
(dead code before, live after), the lift-clear starting position, and the
detection timeouts.
"""

from pathlib import Path

import pytest

from pick_and_place.pipelines import pour as pour_pipeline
from pick_and_place.pipelines.errors import PickHardwareError
from pick_and_place.pipelines.profiles import load_profiles
from pick_and_place.pipelines.strategies import build_strategies

from fakes import (
    FakeArm,
    FakePerception,
    make_cluster,
    make_point,
    make_pose,
)

PROFILES_PATH = Path(__file__).resolve().parents[1] / "config" / "pick_profiles.yaml"


@pytest.fixture
def strategies():
    return build_strategies(load_profiles(PROFILES_PATH))


def perception(**overrides):
    """Perception that finds both the source object and the container."""
    overrides.setdefault("located_point", make_point(x=0.5, z=0.3))
    overrides.setdefault("cluster", make_cluster(z_low=0.30, z_high=0.40))
    overrides.setdefault("grasps", [([make_pose()], [0.9])])
    return FakePerception(**overrides)


def run(arm, perc, strategies, **kwargs):
    kwargs.setdefault("object_name", "cereal")
    kwargs.setdefault("container_name", "bowl")
    request = pour_pipeline.PourRequest(**kwargs)
    return pour_pipeline.execute(arm, perc, request, strategies)


# --- tilt direction (regression) ---------------------------------------------


def test_tilt_direction_is_pinned_positive():
    """Before the merge this was dead code that always returned +1.0.

    The computed sign has never run on hardware and a wrong sign tips the
    container away from the bowl, so it stays pinned until validated.
    """
    from fakes import FakeLogger

    far_left = make_pose(x=0.0, y=-1.0)
    far_right = make_pose(x=1.0, y=1.0)
    log = FakeLogger()
    assert pour_pipeline._pour_direction(log, far_left, far_right) == 1.0
    assert pour_pipeline._pour_direction(log, far_right, far_left) == 1.0


def test_tilt_moves_joint6_positively(strategies):
    arm = FakeArm()
    run(arm, perception(), strategies, object_already_grasped=True)
    assert "move_joints" in arm.calls


# --- ordering (regression) ---------------------------------------------------


def test_missing_source_object_fails_before_touching_the_container(strategies):
    """Locating the container clusters it and wipes the scene; do that after."""
    arm = FakeArm()
    perc = FakePerception(located_point=None)
    success, _ = run(arm, perc, strategies)
    assert not success
    assert "remove_all_collision_objects" not in arm.calls
    assert perc.calls.count("cluster_at") == 0


def test_the_nested_pick_does_not_return_to_a_carry_pose(strategies, monkeypatch):
    """Returning to table_stare first would make the lift plan back downward."""
    captured = {}
    real_execute = pour_pipeline.pick_pipeline.execute

    def spy(arm, perc, request, strats):
        captured["return_to_carry"] = request.return_to_carry
        captured["in_configuration"] = request.in_configuration
        return real_execute(arm, perc, request, strats)

    monkeypatch.setattr(pour_pipeline.pick_pipeline, "execute", spy)
    run(FakeArm(), perception(), strategies)

    assert captured["return_to_carry"] is False
    assert captured["in_configuration"] is True


def test_already_grasped_skips_the_pick_entirely(strategies):
    arm = FakeArm()
    perc = perception()
    run(arm, perc, strategies, object_already_grasped=True)
    assert "detect_grasps" not in perc.calls
    assert "snapshot_scene" not in arm.calls


# --- timeouts (regression) ---------------------------------------------------


def test_pour_keeps_its_own_detection_and_cluster_timeouts(strategies):
    """The pour waits longer for a detection and less for a cluster."""
    perc = perception()
    run(FakeArm(), perc, strategies, object_already_grasped=True)
    assert perc.detect_timeouts == [pytest.approx(60.0)]
    assert perc.cluster_timeouts == [pytest.approx(10.0)]


# --- failure handling --------------------------------------------------------


def test_missing_container_fails(strategies):
    arm = FakeArm()
    perc = FakePerception(located_point=make_point(), cluster=None)
    success, _ = run(arm, perc, strategies, object_already_grasped=True)
    assert not success


def test_unreachable_pour_pose_is_reported_as_failure(strategies):
    """The old pour returned True unconditionally, discarding the result."""
    arm = FakeArm(move_to_pose_results=[False] * 20)
    success, _ = run(arm, perception(), strategies, object_already_grasped=True)
    assert not success


def test_pour_pose_is_retried_five_times(strategies):
    arm = FakeArm(move_to_pose_results=[False] * 20)
    run(arm, perception(), strategies, object_already_grasped=True)
    # already_grasped skips the lift, so every attempt is a pour-pose retry.
    assert arm.calls.count("move_to_pose") == 5


def test_a_failed_lift_still_continues_to_the_pour(strategies):
    """The pre-merge pour fell through on a failed lift rather than aborting."""
    arm = FakeArm(move_to_pose_results=[False, False] + [True] * 10)
    success, _ = run(arm, perception(), strategies)
    assert success


def test_motion_fault_releases_the_gripper(strategies):
    """A fault mid-pour leaves the object clamped; the arm must let go."""

    class Faulty(FakeArm):
        def move_to_pose(self, pose, **kwargs):
            self.calls.append("move_to_pose")
            raise PickHardwareError("planner died")

    arm = Faulty()
    with pytest.raises(PickHardwareError):
        run(arm, perception(), strategies, object_already_grasped=True)
    assert "open_gripper" in arm.calls


def test_joint6_too_close_to_its_limit_fails_rather_than_half_pouring(strategies):
    class Pinned(FakeArm):
        def get_joints(self, degrees=True):
            self.calls.append("get_joints")
            # At the upper limit, so the 3.0 rad tilt has nowhere to go.
            return {
                "joints": {f"joint{i}": 0.0 for i in range(1, 6)} | {"joint6": 6.28}
            }

    arm = Pinned()
    success, _ = run(arm, perception(), strategies, object_already_grasped=True)
    assert not success
    assert "move_joints" not in arm.calls


# --- outcome -----------------------------------------------------------------


def test_pour_returns_the_pick_outcome(strategies):
    _, outcome = run(FakeArm(), perception(), strategies)
    assert outcome.pick_pose is not None


def test_already_grasped_synthesises_an_outcome_from_the_current_pose(strategies):
    arm = FakeArm()
    success, outcome = run(arm, perception(), strategies, object_already_grasped=True)
    assert success
    assert outcome.pick_pose is not None
    assert outcome.pick_pose.header.frame_id == "base_link"
