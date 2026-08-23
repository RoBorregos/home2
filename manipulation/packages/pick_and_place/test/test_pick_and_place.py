"""The pick_and_place invariants worth guarding.

Consolidated from eight files. What survived is what breaks SILENTLY: safety
stops, scene state that leaks into later plans, and config that must fail at
launch rather than mid-descent. Tests that only echoed pick_profiles.yaml were
dropped -- they turned every retune into a test edit and never caught a bug.

These run entirely against fakes. They pin decisions, not geometry: they cannot
tell you the arm reaches anything, and a green run is not a working robot.
They also need the ROS message packages, so they only run inside the container.
"""

import inspect
import pytest
import yaml

from fakes import (
    FakeArm,
    FakeFlatResponse,
    FakeLogger,
    FakePerception,
    make_cluster,
    make_point,
    make_pose,
)
from frida_constants.manipulation_constants import (
    BOWL_NAME,
    RIM_NAMES,
)
from frida_interfaces.msg import ManipulationTask, PlaceParams
from pathlib import Path
from pick_and_place.manipulation_core import ManipulationCore
from pick_and_place.pipelines import (
    pick as pick_pipeline,
    place as place_pipeline,
    pour as pour_pipeline,
)
from pick_and_place.pipelines.classification import (
    PICK_STRATEGY_BOWL,
    PICK_STRATEGY_GPD,
    resolve_pick_strategy,
)
from pick_and_place.pipelines.errors import (
    PickAborted,
    PickAttemptFailed,
    PickHardwareError,
)
from pick_and_place.pipelines.profiles import ProfileError, load_profiles
from pick_and_place.pipelines.strategies import (
    GraspCandidate,
    PickOutcome,
    build_strategies,
)
from pick_and_place.robot.arm import AttachResult, ContactResult, RobotArm
from pick_and_place.robot.perception import Perception

PROFILES_PATH = Path(__file__).resolve().parents[1] / "config" / "pick_profiles.yaml"


# ============================================================================
# strategies
# ============================================================================


@pytest.fixture
def strategies():
    return build_strategies(load_profiles(PROFILES_PATH))


def candidate(z=0.3, score=0.9):
    return GraspCandidate(
        pose=make_pose(z=z), score=score, pose_index=0, alternative_index=0
    )


def test_build_rejects_a_profile_with_no_registered_class():
    profiles = load_profiles(PROFILES_PATH)
    profiles["invented"] = profiles["gpd"]
    with pytest.raises(KeyError, match="No strategy registered"):
        build_strategies(profiles)


def test_self_collision_skips_the_candidate_before_touching_the_robot(strategies):
    arm = FakeArm(self_collides=[True])
    with pytest.raises(PickAttemptFailed, match="self-collides"):
        strategies["rim"].attempt(arm, candidate())
    assert arm.calls == ["endpoint_self_collides"]


def test_incomplete_descent_fails_the_attempt(strategies):
    arm = FakeArm(descent_results=[False])
    with pytest.raises(PickAttemptFailed, match="did not complete"):
        strategies["rim"].attempt(arm, candidate())
    assert "close_gripper" not in arm.calls


def test_estop_during_descent_aborts_rather_than_retrying(strategies):
    arm = FakeArm(abort_after=0)
    with pytest.raises(PickAborted):
        strategies["rim"].attempt(arm, candidate())


def test_flat_retract_is_always_above_the_contact_point(strategies):
    for descended in (0.09, 0.12, 0.15):
        arm = FakeArm(
            contact_results=[ContactResult(contact=True, descended_m=descended)]
        )
        strategies["flat"].attempt(arm, candidate(z=0.30))
        contact_z = 0.45 - descended
        assert arm.poses["move_to_pose"][1].pose.position.z > contact_z


def test_gpd_still_reports_the_pick_when_attaching_fails(strategies):
    """The grasp succeeded; only the scene bookkeeping did not."""
    arm = FakeArm(attach_result=AttachResult(attached=False))
    outcome = strategies["gpd"].attempt(arm, candidate(score=0.6))
    assert outcome.grasp_score == pytest.approx(0.6)
    assert outcome.object_pick_height == 0.0
    assert arm.logger.error_messages


# ============================================================================
# pick_pipeline
# ============================================================================


def flat_perception(z=0.5):
    return FakePerception(flat_response=FakeFlatResponse(make_pose(z=z)))


def run_pick(arm, perception, strategies, **kwargs):
    request = pick_pipeline.PickRequest(**kwargs)
    return pick_pipeline.execute(arm, perception, request, strategies)


def test_abort_propagates_out_of_the_pipeline(strategies):
    arm = FakeArm(abort_after=0)
    with pytest.raises(PickAborted):
        run_pick(arm, flat_perception(), strategies, object_name="basket")


def test_gpd_falls_back_to_the_next_config(strategies):
    """First config yields nothing; the second must still be tried."""
    perception = FakePerception(
        located_point=make_point(),
        cluster=make_cluster(),
        grasps=[([], []), ([make_pose()], [0.9])],
    )
    arm = FakeArm()
    success, _ = run_pick(arm, perception, strategies, object_name="cup")
    assert success
    assert perception.calls.count("detect_grasps") == 2


def test_tip_offset_uses_the_per_strategy_parameter(strategies):
    """rim_tip_offset is -0.18, and the rim pre-grasp sits 0.10 above."""
    arm = FakeArm()
    run_pick(arm, flat_perception(z=0.50), strategies, object_name="basket")
    assert arm.poses["move_to_pregrasp"][0].pose.position.z == pytest.approx(0.42)


# ============================================================================
# place_pipeline
# ============================================================================


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


def place_perception(**overrides):
    overrides.setdefault("heatmap_point", make_point(z=0.70))
    overrides.setdefault("surface_cloud", make_cluster())
    return FakePerception(**overrides)


def run_place(arm, perc, place_params, outcome=None):
    return place_pipeline.execute(arm, perc, place_params, outcome or held())


def held(height=0.12):
    """An outcome from a successful pick, as place receives it."""
    return PickOutcome(pick_pose=make_pose(z=0.5), object_pick_height=height)


def test_shelf_guard_is_added_and_always_removed():
    arm = FakeArm()
    run_place(arm, place_perception(), params(is_shelf=True))
    assert "add_shelf_ceiling_guard" in arm.calls
    assert "remove_shelf_ceiling_guard" in arm.calls
    assert arm.calls.index("add_shelf_ceiling_guard") < arm.calls.index(
        "remove_shelf_ceiling_guard"
    )


def test_guard_is_removed_even_when_the_return_fails():
    arm = FakeArm(named_position_results=[False] * 10)
    run_place(arm, place_perception(), params(is_shelf=True))
    assert "remove_shelf_ceiling_guard" in arm.calls


def test_success_detaches_then_opens_the_gripper():
    arm = FakeArm()
    run_place(arm, place_perception(), params())
    assert arm.calls.index("detach_pick_objects") < arm.calls.index("open_gripper")


def test_place_fails_when_no_rung_is_reachable():
    arm = FakeArm(move_to_pose_results=[False] * 8)
    assert run_place(arm, place_perception(), params(is_shelf=False)) is False
    assert "detach_pick_objects" not in arm.calls


# ============================================================================
# pour_pipeline
# ============================================================================


def pour_perception(**overrides):
    """Perception that finds both the source object and the container."""
    overrides.setdefault("located_point", make_point(x=0.5, z=0.3))
    overrides.setdefault("cluster", make_cluster(z_low=0.30, z_high=0.40))
    overrides.setdefault("grasps", [([make_pose()], [0.9])])
    return FakePerception(**overrides)


def run_pour(arm, perc, strategies, **kwargs):
    kwargs.setdefault("object_name", "cereal")
    kwargs.setdefault("container_name", "bowl")
    request = pour_pipeline.PourRequest(**kwargs)
    return pour_pipeline.execute(arm, perc, request, strategies)


def test_tilt_direction_follows_the_container():
    """A positive joint6 rotation tilts along -local_X.

    make_pose has identity orientation, so local_X is world +X: a container on
    the gripper's -X side must pour positive and one on its +X side negative.
    This was dead code returning +1.0 either way until the poses stopped being
    the same object; pinning it here is what keeps it from silently dying again.
    """
    from fakes import FakeLogger

    log = FakeLogger()
    gripper = make_pose(x=0.5, y=0.0)
    assert pour_pipeline._pour_direction(log, make_pose(x=0.0, y=0.0), gripper) == 1.0
    assert pour_pipeline._pour_direction(log, make_pose(x=1.0, y=0.0), gripper) == -1.0


def test_tilt_direction_defaults_positive_when_undetermined():
    """Coincident poses give no direction; keep the historical +1.0 default."""
    from fakes import FakeLogger

    log = FakeLogger()
    same = make_pose(x=0.5, y=0.0)
    assert pour_pipeline._pour_direction(log, same, make_pose(x=0.5, y=0.0)) == 1.0


def test_motion_fault_releases_the_gripper(strategies):
    """A fault mid-pour leaves the object clamped; the arm must let go."""

    class Faulty(FakeArm):
        def move_to_pose(self, pose, **kwargs):
            self.calls.append("move_to_pose")
            raise PickHardwareError("planner died")

    arm = Faulty()
    with pytest.raises(PickHardwareError):
        run_pour(arm, pour_perception(), strategies, object_already_grasped=True)
    assert "open_gripper" in arm.calls


def test_joint6_too_close_to_its_limit_fails_rather_than_half_pouring(strategies):
    # Read the limit the pipeline actually bound. conftest only stubs
    # frida_pymoveit2 when it is NOT installed, so the real xarm6 limits
    # (+-pi*0.99) apply inside the container while the stub uses +-6.28.
    # A hardcoded value sits at the limit in only one of the two.
    upper = pour_pipeline.JOINT_POSITION_LIMITS["joint6"][1]

    class Pinned(FakeArm):
        def get_joints(self, degrees=True):
            self.calls.append("get_joints")
            # At the upper limit, so the 3.0 rad tilt has nowhere to go.
            return {
                "joints": {f"joint{i}": 0.0 for i in range(1, 6)} | {"joint6": upper}
            }

    arm = Pinned()
    success, _ = run_pour(
        arm, pour_perception(), strategies, object_already_grasped=True
    )
    assert not success
    assert "move_joints" not in arm.calls


def test_the_nested_pick_does_not_return_to_a_carry_pose(strategies, monkeypatch):
    """Returning to table_stare first would make the lift plan back downward."""
    captured = {}
    real_execute = pour_pipeline.pick_pipeline.execute

    def spy(arm, perc, request, strats):
        captured["return_to_carry"] = request.return_to_carry
        captured["in_configuration"] = request.in_configuration
        return real_execute(arm, perc, request, strats)

    monkeypatch.setattr(pour_pipeline.pick_pipeline, "execute", spy)
    run_pour(FakeArm(), pour_perception(), strategies)

    assert captured["return_to_carry"] is False
    assert captured["in_configuration"] is True


# ============================================================================
# manipulation_core
# ============================================================================


@pytest.fixture
def core():
    node = ManipulationCore.__new__(ManipulationCore)
    node.arm = FakeArm()
    node.perception = FakePerception()
    node.strategies = {}
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node._last_pick = PickOutcome()
    node._pipelines = {
        ManipulationTask.PICK: node._run_pick,
        ManipulationTask.PICK_CLOSEST: node._run_pick_closest,
        ManipulationTask.PLACE: node._run_place,
        ManipulationTask.POUR: node._run_pour,
    }
    return node


class _Request:
    """A ManipulationAction goal request, only the fields the node reads."""

    def __init__(self, task_type=ManipulationTask.PICK, **params):
        self.task_type = task_type
        self.scan_environment = params.pop("scan_environment", False)
        self.pick_params = type(
            "P",
            (),
            {
                "object_name": "cup",
                "object_point": None,
                "min_distance": 0.0,
                "max_distance": 1.0,
                "in_configuration": False,
            },
        )()
        self.place_params = type("P", (), {"is_shelf": False})()
        self.pour_params = type(
            "P",
            (),
            {
                "object_name": "cereal",
                "bowl_name": "bowl",
                "object_already_grasped": False,
            },
        )()


def test_every_task_type_has_a_pipeline(core):
    for task in (
        ManipulationTask.PICK,
        ManipulationTask.PICK_CLOSEST,
        ManipulationTask.PLACE,
        ManipulationTask.POUR,
    ):
        assert task in core._pipelines


def test_a_failed_pick_clears_the_remembered_outcome(core, monkeypatch):
    core._last_pick = PickOutcome(pick_pose=make_pose(), object_pick_height=0.12)
    monkeypatch.setattr(
        "pick_and_place.pipelines.pick.execute",
        lambda *a, **k: (False, PickOutcome()),
    )
    assert core._run_pick(_Request()) is False
    assert core._last_pick.pick_pose is None


def test_a_raising_pour_does_not_leave_a_stale_outcome_behind(core, monkeypatch):
    """A later place would otherwise use a previous task's drop height."""
    core._last_pick = PickOutcome(pick_pose=make_pose(), object_pick_height=0.30)

    def boom(*args, **kwargs):
        raise PickHardwareError("planner died")

    monkeypatch.setattr("pick_and_place.pipelines.pour.execute", boom)
    with pytest.raises(PickHardwareError):
        core._run_pour(_Request(ManipulationTask.POUR))
    assert core._last_pick.pick_pose is None


# ============================================================================
# profiles
# ============================================================================


def _load_with(tmp_path, profile: str, field: str, value):
    """Carga el YAML real con un solo campo alterado.

    Sustituye al parámetro `overrides` que tenía load_profiles: la validación se
    prueba inyectando el valor malo por el mismo camino que usa el nodo.
    ``field`` admite notación anidada: "force_guard.jump_trip".
    """
    document = yaml.safe_load(PROFILES_PATH.read_text())
    target = document[profile]
    *parents, leaf = field.split(".")
    for step in parents:
        target = target[step]
    target[leaf] = value
    written = tmp_path / "profiles.yaml"
    written.write_text(yaml.safe_dump(document))
    return load_profiles(written)


@pytest.fixture
def profiles():
    return load_profiles(PROFILES_PATH)


def test_every_strategy_key_has_a_profile(profiles):
    from pick_and_place.pipelines.classification import PICK_STRATEGY_KEYS

    assert set(profiles) == set(PICK_STRATEGY_KEYS)


@pytest.mark.parametrize(
    "profile,field,value,fragment",
    [
        ("bowl", "grasp_z_tweak", 0.5, "must be positive"),
        ("flat", "force_guard.jump_trip", 99.0, "hard_ceiling"),
        ("flat", "force_guard.timeout", 2.0, "less than pre_grasp_height"),
        ("gpd", "num_alternatives", 0, "num_alternatives"),
        ("rim", "pre_grasp_height", "abc", "must be a float"),
        ("rim", "descent_speed", 0.0, "descent_speed"),
    ],
)
def test_invalid_values_are_rejected_at_load_time(
    tmp_path, profile, field, value, fragment
):
    with pytest.raises(ProfileError, match=fragment):
        _load_with(tmp_path, profile, field, value)


def test_unknown_key_in_profile_is_rejected(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        "gpd:\n"
        "  strategy: direct\n"
        "  tip_offset_param: ee_link_offset\n"
        "  num_alternatives: 1\n"
        "  alternative_step: 0.0\n"
        "  pre_grasp_height: 0.0\n"
        "  pre_grasp_velocity: 0.3\n"
        "  close_settle: 1.5\n"
        "  validate_endpoint: false\n"
        "  lift_after_grasp: false\n"
        "  attach_collision_object: true\n"
        "  typo_field: 1.0\n"
    )
    with pytest.raises(ProfileError, match="unknown keys"):
        load_profiles(bad)


def test_missing_required_key_is_rejected(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("gpd:\n  strategy: direct\n")
    with pytest.raises(ProfileError, match="missing required keys"):
        load_profiles(bad)


# ============================================================================
# classification
# ============================================================================


def test_bowl_wins_over_rim():
    """BOWL_NAME is a member of RIM_NAMES, so ordering decides the outcome.

    If rim were tested first, every bowl would get the rim profile and descend
    0.15 m instead of 0.08 m -- straight through the bowl.
    """
    assert BOWL_NAME in RIM_NAMES
    assert resolve_pick_strategy(BOWL_NAME) == PICK_STRATEGY_BOWL


@pytest.mark.parametrize("name", ["", None, "unknown_object", "cereal", "milk"])
def test_unknown_names_fall_back_to_gpd(name):
    assert resolve_pick_strategy(name) == PICK_STRATEGY_GPD


# ============================================================================
# fakes
# ============================================================================


def _public_api(cls):
    return {
        name
        for name, member in vars(cls).items()
        if not name.startswith("_")
        and (inspect.isfunction(member) or isinstance(member, property))
    }


def test_fake_arm_covers_the_real_arm():
    missing = _public_api(RobotArm) - set(dir(FakeArm))
    assert not missing, f"FakeArm is missing: {sorted(missing)}"


def test_fake_perception_covers_the_real_perception():
    missing = _public_api(Perception) - set(dir(FakePerception))
    assert not missing, f"FakePerception is missing: {sorted(missing)}"


def test_pipelines_do_not_import_ros_clients():
    """The pipelines must go through the facades, not talk ROS directly.

    This is the layering rule the refactor exists to enforce; without a test it
    erodes the first time someone needs "just one more service call".
    """
    import pathlib

    pipelines = pathlib.Path(inspect.getfile(RobotArm)).parent.parent / "pipelines"
    offenders = []
    for path in pipelines.glob("*.py"):
        source = path.read_text()
        for banned in ("ActionClient", "create_client", "create_subscription"):
            if banned in source:
                offenders.append(f"{path.name}: {banned}")
    assert not offenders, f"pipelines must not create ROS clients: {offenders}"
