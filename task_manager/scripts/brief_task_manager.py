#!/usr/bin/env python3
"""
Generic brief-driven task manager.

One entry point for every task whose brief needs no custom guards or event wiring.
Pick & Place has its own script because it adds object-routing logic; this covers the
rest, and is the template for any new task.

    ros2 run task_manager brief_task_manager.py --ros-args -p brief:=doing_laundry
    ros2 run task_manager brief_task_manager.py --ros-args -p brief:=restaurant
"""

import os
import sys

from task_manager.executor.brief_runner import BriefRunner, spin_brief
from task_manager.planner.brief import briefs_dir, load_brief
from task_manager.utils.subtask_manager import Task

# brief name -> the Task enum its subtask managers expect
TASKS = {
    "pick_and_place": Task.PICK_AND_PLACE,
    "doing_laundry": Task.DOING_LAUNDRY,
    "hric": Task.HRIC,
    "restaurant": Task.RESTAURANT,
}


class GenericBriefTM(BriefRunner):
    def __init__(self, brief_name: str):
        brief = load_brief(os.path.join(briefs_dir(), f"{brief_name}.yaml"))
        super().__init__(
            node_name=f"{brief_name}_brief_tm",
            task=TASKS.get(brief_name, Task.DEBUG),
            brief=brief,
            manifest_path=os.environ.get("FRIDA_CAPABILITIES", ""),
        )
        self._wire_triggers()

    def _wire_triggers(self) -> None:
        """
        Connect declared events to their real sources.

        Only the doorbell has a wired detector today (hri.arm_door_detection). The rest
        are declared in the briefs and fire manually until their detectors exist, which
        keeps the gap visible instead of hiding it behind a silent no-op.
        """
        events = {trigger.on for trigger in self.brief.triggers}
        if "doorbell" in events:
            try:
                self.subtask_manager.hri.arm_door_detection(True)
            except Exception as error:  # noqa: BLE001
                self.get_logger().warning(f"could not arm doorbell detection: {error}")
        unwired = events - {"doorbell"}
        if unwired:
            self.get_logger().warning(
                f"triggers with no detector yet, fire them manually: {sorted(unwired)}"
            )

    def run(self) -> None:
        # poll the one event source that exists; others arrive via fire_trigger()
        if self.current_state not in {"wait_start", "done"}:
            try:
                if getattr(self.subtask_manager.hri, "door_event_detected", False):
                    self.fire_trigger("doorbell")
                    self.subtask_manager.hri.door_event_detected = False
            except Exception:  # noqa: BLE001 — event polling must never stop the run
                pass
        super().run()


def _brief_name(argv) -> str:
    for index, value in enumerate(argv):
        if value == "-p" and index + 1 < len(argv) and argv[index + 1].startswith("brief:="):
            return argv[index + 1].split(":=", 1)[1]
        if value.startswith("brief:="):
            return value.split(":=", 1)[1]
    return os.environ.get("FRIDA_BRIEF", "doing_laundry")


def main(args=None) -> None:
    name = _brief_name(sys.argv)
    spin_brief(lambda: GenericBriefTM(name), args=args)


if __name__ == "__main__":
    main()
