"""The object_name -> pick strategy classifier.

Guards the drift that motivated the refactor: perception and motion used to
derive these predicates independently.
"""

import pytest
from frida_constants.manipulation_constants import (
    BOWL_NAME,
    FLAT_OBJECT_NAMES,
    PEAK_NAMES,
    RIM_NAMES,
)

from pick_and_place.pipelines.classification import (
    PICK_STRATEGY_BOWL,
    PICK_STRATEGY_FLAT,
    PICK_STRATEGY_GPD,
    PICK_STRATEGY_KEYS,
    PICK_STRATEGY_PEAK,
    PICK_STRATEGY_RIM,
    resolve_pick_strategy,
)


def test_bowl_wins_over_rim():
    """BOWL_NAME is a member of RIM_NAMES, so ordering decides the outcome.

    If rim were tested first, every bowl would get the rim profile and descend
    0.15 m instead of 0.08 m -- straight through the bowl.
    """
    assert BOWL_NAME in RIM_NAMES
    assert resolve_pick_strategy(BOWL_NAME) == PICK_STRATEGY_BOWL


@pytest.mark.parametrize("name", [n for n in RIM_NAMES if n != BOWL_NAME])
def test_rim_names_resolve_to_rim(name):
    assert resolve_pick_strategy(name) == PICK_STRATEGY_RIM


@pytest.mark.parametrize("name", PEAK_NAMES)
def test_peak_names_resolve_to_peak(name):
    assert resolve_pick_strategy(name) == PICK_STRATEGY_PEAK


@pytest.mark.parametrize("name", FLAT_OBJECT_NAMES)
def test_flat_names_resolve_to_flat(name):
    """Every flat object shares one strategy: forks and plates alike."""
    assert resolve_pick_strategy(name) == PICK_STRATEGY_FLAT


@pytest.mark.parametrize("name", ["", None, "unknown_object", "cereal", "milk"])
def test_unknown_names_fall_back_to_gpd(name):
    assert resolve_pick_strategy(name) == PICK_STRATEGY_GPD


@pytest.mark.parametrize("name", ["FORK", "Bowl", "LAUNDRY_BASKET", "Clothes", "Plate"])
def test_classification_is_case_insensitive(name):
    assert resolve_pick_strategy(name) != PICK_STRATEGY_GPD


def test_flat_set_covers_utensils_and_larger_flat_items():
    """The set is a single flat class -- there is no cutlery/non-cutlery split."""
    for name in ("fork", "knife", "spoon", "cutlery"):
        assert name in FLAT_OBJECT_NAMES
    for name in ("plate", "red_plate", "toothpaste", "sponge", "dishwasher_tab"):
        assert name in FLAT_OBJECT_NAMES
    assert len(FLAT_OBJECT_NAMES) == len(set(FLAT_OBJECT_NAMES))


def test_every_strategy_key_is_reachable():
    """Each key must be produced by some object name, or its profile is dead."""
    reachable = {resolve_pick_strategy(n) for n in FLAT_OBJECT_NAMES + RIM_NAMES}
    reachable |= {resolve_pick_strategy(n) for n in PEAK_NAMES}
    reachable.add(resolve_pick_strategy("something_unseen"))
    assert reachable == set(PICK_STRATEGY_KEYS)
