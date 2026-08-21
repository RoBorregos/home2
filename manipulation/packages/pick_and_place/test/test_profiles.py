"""Profile loading, override and validation.

These tests need no ROS message packages, so they run anywhere.
"""

import math
from pathlib import Path

import pytest
import yaml

from pick_and_place.pipelines.profiles import (
    ProfileError,
    load_profiles,
)

PROFILES_PATH = Path(__file__).resolve().parents[1] / "config" / "pick_profiles.yaml"


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
    "name,expected",
    [
        # These are the values the pre-refactor constants produced:
        #   RIM_DESCENT_DISTANCE  = RIM_PRE_GRASP_HEIGHT - RIM_GRASP_Z_TWEAK
        #   BOWL_DESCENT_DISTANCE = 0.08 (BOWL_GRASP_Z_TWEAK derived from it)
        #   peak descended exactly PEAK_PRE_GRASP_HEIGHT
        ("rim", 0.15),
        ("bowl", 0.08),
        ("peak", 0.15),
    ],
)
def test_descent_distance_matches_pre_refactor_constants(profiles, name, expected):
    assert math.isclose(
        profiles[name].effective_descent_distance, expected, abs_tol=1e-9
    )


def test_tip_offset_params_are_per_strategy(profiles):
    assert profiles["rim"].tip_offset_param == "rim_tip_offset"
    assert profiles["bowl"].tip_offset_param == "bowl_tip_offset"
    # Peak and GPD both fell into the `else` branch that used ee_link_offset.
    assert profiles["peak"].tip_offset_param == "ee_link_offset"
    assert profiles["gpd"].tip_offset_param == "ee_link_offset"


def test_candidate_counts_match_pre_refactor_branching(profiles):
    assert (
        profiles["flat"].num_alternatives,
        profiles["flat"].alternative_step,
    ) == (
        6,
        -0.005,
    )
    assert (profiles["gpd"].num_alternatives, profiles["gpd"].alternative_step) == (
        2,
        -0.025,
    )
    for name in ("rim", "bowl", "peak"):
        assert profiles[name].num_alternatives == 1


def test_flat_force_guard_matches_pre_refactor_constants(profiles):
    guard = profiles["flat"].force_guard
    assert guard is not None
    assert guard.descent_speed == 12.0
    assert guard.timeout == 16.0
    assert guard.grace_period == 0.3
    assert guard.contact_settle == 1.0
    assert guard.min_contact_descent == 0.08
    assert guard.ignore_joints == (0, 5)
    assert guard.jump_window == 0.3
    assert guard.jump_trip == 2.5
    assert guard.jump_sustain == 3
    assert guard.hard_ceiling == 15.0
    assert profiles["flat"].post_contact_retract == 0.004
    assert profiles["flat"].pre_grasp_height == 0.15


def test_only_gpd_attaches_a_collision_object(profiles):
    assert profiles["gpd"].attach_collision_object is True
    assert not any(
        profiles[name].attach_collision_object
        for name in ("flat", "rim", "bowl", "peak")
    )


def test_only_flat_lifts(profiles):
    assert profiles["flat"].lift_after_grasp is True
    assert not any(
        profiles[name].lift_after_grasp for name in ("rim", "bowl", "peak", "gpd")
    )


def test_only_descent_strategies_validate_the_endpoint(profiles):
    for name in ("rim", "bowl", "peak"):
        assert profiles[name].validate_endpoint is True
    # The flat branch never ran a self-collision pre-check.
    assert profiles["flat"].validate_endpoint is False


def test_derived_descent_follows_the_tweak(tmp_path):
    """descent = pre_grasp_height - grasp_z_tweak, recomputado al cambiar uno."""
    profiles = _load_with(tmp_path, "bowl", "grasp_z_tweak", 0.03)
    assert math.isclose(profiles["bowl"].effective_descent_distance, 0.07, abs_tol=1e-9)


@pytest.mark.parametrize(
    "profile,field,value,fragment",
    [
        # A tweak above the pre-grasp height would drive the arm upward.
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


def test_missing_file_is_reported_clearly():
    with pytest.raises(ProfileError, match="not found"):
        load_profiles(Path("/nonexistent/pick_profiles.yaml"))


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
