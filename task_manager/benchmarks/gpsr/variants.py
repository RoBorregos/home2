"""Compatibility imports for the paper's scheduling benchmarks."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_manager.gpsr.merger import (  # noqa: E402
    NoInterleavingSchedule,
    _optimal_schedule as variant_schedule,
    merge as merge_variant,
)

__all__ = ["NoInterleavingSchedule", "variant_schedule", "merge_variant"]
