#!/usr/bin/env python3
"""
Pick & Place on the brief-driven executor.

Runs alongside the original pickandplace_task_manager.py rather than replacing it —
the old manager stays the fallback until this one wins a timed rehearsal. Everything
task-specific lives in briefs/pick_and_place.yaml and the guards below.

    ros2 run task_manager ppc_brief_task_manager.py
    ros2 run task_manager ppc_brief_task_manager.py --ros-args -p posture:=aggressive
"""

import os

from task_manager.executor.brief_runner import BriefRunner, spin_brief
from task_manager.planner.brief import Brief, briefs_dir, load_brief
from task_manager.utils.subtask_manager import Task

# The plate needs a flat grasp the arm does not do reliably, so its 100-point pick is
# not worth a whole cycle. Tracked in docs/task_manager/ppc/time_strategy_2026.md.
UNGRASPABLE = {"red_plate", "plate", "dish"}

# Category -> where it belongs, from the rulebook's designated locations.
# Trash is announced during setup days, hence the parameter rather than a constant.
DESTINATIONS = {
    "cutlery": "dishwasher",
    "tableware": "dishwasher",
    "dish": "dishwasher",
    "trash": "trash",
}
DEFAULT_DESTINATION = "cabinet"


def graspable(binding: dict, world) -> bool:
    """Guard referenced by the brief's pick_place objective."""
    target = binding.get("obj")
    if target is None:
        return True
    return target.name not in UNGRASPABLE


class PickAndPlaceBriefTM(BriefRunner):
    def __init__(self):
        brief = _load_brief()
        super().__init__(
            node_name="pick_and_place_brief_tm",
            task=Task.PICK_AND_PLACE,
            brief=brief,
            manifest_path=_manifest_path(),
            guards={"graspable": graspable},
        )
        self.declare_parameter("trash_category", "fruit")
        self.trash_category = (
            self.get_parameter("trash_category").get_parameter_value().string_value
        )

    def _destination_for(self, label: str) -> str:
        """Route a detected object to its designated furniture."""
        category = self.subtask_manager.hri.deterministic_categorization(label)
        if category == self.trash_category:
            return "trash"
        return DESTINATIONS.get(category, DEFAULT_DESTINATION)


def _load_brief() -> Brief:
    return load_brief(os.path.join(briefs_dir(), "pick_and_place.yaml"))


def _manifest_path() -> str:
    return os.environ.get(
        "FRIDA_CAPABILITIES",
        os.path.join(os.path.dirname(briefs_dir()), "config", "capabilities.yaml"),
    )


def main(args=None) -> None:
    spin_brief(PickAndPlaceBriefTM, args=args)


if __name__ == "__main__":
    main()
