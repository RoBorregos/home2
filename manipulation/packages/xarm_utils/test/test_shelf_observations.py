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
