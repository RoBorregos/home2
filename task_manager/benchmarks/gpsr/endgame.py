"""Compatibility imports for the deadline-planning benchmark."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from task_manager.gpsr.endgame import EndgamePlan, plan_endgame  # noqa: E402

__all__ = ["EndgamePlan", "plan_endgame"]
