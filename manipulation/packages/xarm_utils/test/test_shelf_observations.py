"""Regressions for incomplete and unstable shelf observations over time."""

import pytest

from xarm_utils.shelf_observations import ShelfLevelConsensus


def test_partial_views_confirm_repeated_levels():
    consensus = ShelfLevelConsensus()
    result = None
    for i in range(12):
        levels = [0.93 + (i % 2) * 0.001]
        if i % 3 != 1:
            levels.append(1.30 + (i % 2) * 0.001)
        result = consensus.add(levels)
        if i < 11:
            assert result is None
    assert result == pytest.approx([0.9305, 1.3005])


def test_isolated_object_height_does_not_become_a_level():
    consensus = ShelfLevelConsensus()
    for i in range(12):
        result = consensus.add([0.93, 1.30] + ([0.73] if i < 2 else []))
    assert result == pytest.approx([0.93, 1.30])


def test_one_repeated_level_is_insufficient():
    consensus = ShelfLevelConsensus()
    for _ in range(20):
        assert consensus.add([0.93]) is None


def test_height_drift_does_not_pass_as_stable():
    consensus = ShelfLevelConsensus()
    for i in range(12):
        result = consensus.add([0.93, 1.29 + i * 0.004])
    assert result is None


def test_old_observations_expire_from_window():
    consensus = ShelfLevelConsensus()
    for _ in range(12):
        consensus.add([0.93, 1.30])
    for _ in range(12):
        result = consensus.add([0.70, 1.10])
    assert result == pytest.approx([0.70, 1.10])


def test_missing_geometry_counts_against_confirmation():
    consensus = ShelfLevelConsensus()
    for i in range(12):
        result = consensus.add([0.93, 1.30] if i < 5 else [])
    assert result is None


def strong_evidence(**changes):
    data = dict(
        quality_score=0.95,
        width_coverage=0.95,
        depth_coverage=0.9,
        residual_p90_m=0.002,
        boundary_count=3,
        normal=[0, 0, 1],
        calibration_warning=False,
    )
    data.update(changes)
    return data


def test_good_structural_evidence_confirms_after_six_frames():
    consensus = ShelfLevelConsensus()
    for i in range(6):
        result = consensus.add([0.93, 1.30], [strong_evidence(), strong_evidence()])
        if i < 5:
            assert result is None
    assert result == pytest.approx([0.93, 1.30])
    assert consensus.confirmation_mode == "quality"


@pytest.mark.parametrize(
    "change",
    [
        {"boundary_count": 2},
        {"width_coverage": 0.3},
        {"calibration_warning": True},
        {"quality_score": float("nan")},
    ],
)
def test_incomplete_or_questionable_geometry_keeps_full_window(change):
    consensus = ShelfLevelConsensus()
    for i in range(12):
        result = consensus.add(
            [0.93, 1.30], [strong_evidence(**change), strong_evidence(**change)]
        )
        if i < 11:
            assert result is None
    assert result == pytest.approx([0.93, 1.30])
    assert consensus.confirmation_mode == "full_window"


def test_unresolved_extra_surface_defers_early_confirmation():
    consensus = ShelfLevelConsensus()
    for i in range(6):
        heights = [0.93, 1.3] + ([1.6] if i >= 4 else [])
        result = consensus.add(heights, [strong_evidence() for _ in heights])
    assert result is None


def test_normal_changes_defer_early_confirmation():
    import math

    consensus = ShelfLevelConsensus()
    for i in range(6):
        angle = math.radians(5 if i % 2 else -5)
        quality = strong_evidence(normal=[math.sin(angle), 0, math.cos(angle)])
        result = consensus.add([0.93, 1.3], [quality, quality])
    assert result is None


def test_reset_forgets_quality_and_height_together():
    consensus = ShelfLevelConsensus()
    for _ in range(5):
        consensus.add([0.93, 1.3], [strong_evidence(), strong_evidence()])
    consensus.reset()
    assert not consensus.samples and not consensus.qualities
    assert consensus.add([0.93, 1.3], [strong_evidence(), strong_evidence()]) is None


def test_quality_metadata_follows_sorted_heights():
    consensus = ShelfLevelConsensus()
    for _ in range(6):
        result = consensus.add(
            [1.3, 0.93], [strong_evidence(), strong_evidence(width_coverage=0.2)]
        )
    assert result is None


def test_quality_length_mismatch_does_not_change_window():
    consensus = ShelfLevelConsensus()
    with pytest.raises(ValueError, match="match"):
        consensus.add([0.93, 1.3], [strong_evidence()])
    assert not consensus.samples
