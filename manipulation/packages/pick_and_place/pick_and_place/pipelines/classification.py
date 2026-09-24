"""Which pick strategy handles which object.

The single source of truth for `object_name -> strategy`. The pick pipeline and
the strategy table both resolve through here, so they cannot disagree about how
an object is picked.

The keys double as the profile keys in ``config/pick_profiles.yaml``; a mismatch
between the two fails at node startup (see ``strategies.build_strategies``).
"""

from frida_constants.manipulation_constants import (
    BOWL_NAME,
    FLAT_OBJECT_NAMES,
    PEAK_NAMES,
    RIM_NAMES,
)

PICK_STRATEGY_GPD = "gpd"
PICK_STRATEGY_FLAT = "flat"
PICK_STRATEGY_RIM = "rim"
PICK_STRATEGY_BOWL = "bowl"
PICK_STRATEGY_PEAK = "peak"

PICK_STRATEGY_KEYS = (
    PICK_STRATEGY_FLAT,
    PICK_STRATEGY_RIM,
    PICK_STRATEGY_BOWL,
    PICK_STRATEGY_PEAK,
    PICK_STRATEGY_GPD,
)

# Order matters: BOWL_NAME is a member of RIM_NAMES, so bowl must be tested
# before rim, or every bowl would resolve to the rim profile and descend 0.15 m
# instead of 0.08 m -- straight through the bowl.
_STRATEGY_BY_OBJECT_NAME = (
    (PICK_STRATEGY_BOWL, frozenset({BOWL_NAME})),
    (PICK_STRATEGY_PEAK, frozenset(PEAK_NAMES)),
    (PICK_STRATEGY_RIM, frozenset(RIM_NAMES)),
    (PICK_STRATEGY_FLAT, frozenset(FLAT_OBJECT_NAMES)),
)


def resolve_pick_strategy(object_name: str) -> str:
    """Map an object name to the pick strategy that should handle it.

    Unknown objects fall back to the GPD strategy, which is the historical
    default.
    """
    if not object_name:
        return PICK_STRATEGY_GPD
    key = object_name.lower()
    for strategy, names in _STRATEGY_BY_OBJECT_NAME:
        if key in names:
            return strategy
    return PICK_STRATEGY_GPD
