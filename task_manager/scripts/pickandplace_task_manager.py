#!/usr/bin/env python3

"""
Pick and Place Challenge as a behaviour tree driven by frida_constants/data/tasks/ppc.json.
Debug: -p start_at:=cleanup|breakfast -p state_file:=<snapshot.json>
-p mock_areas:=navigation,vision -p step:=true -p spec_file:=<ppc.json>
"""

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from task_manager.utils.bt import Action, ForEach, RunLog, TreeRunner, optional, sequence
from task_manager.utils.colored_logger import CLog
from task_manager.utils.status import Status
from task_manager.utils.task import Task

OK = Status.EXECUTION_SUCCESS


@dataclass
class Item:
    name: str
    status: str = "pending"  # pending | picked | placed | skipped


@dataclass
class PPCState:
    """What the task remembers; dumped after every leaf and loadable with state_file."""

    started_at: float | None = None
    location: str | None = None
    holding: str | None = None
    objects: list = field(default_factory=list)
    breakfast: list = field(default_factory=list)

    @classmethod
    def from_dict(cls, d: dict) -> "PPCState":
        d = dict(d)
        d["objects"] = [Item(**o) for o in d.get("objects", [])]
        d["breakfast"] = [Item(**b) for b in d.get("breakfast", [])]
        return cls(**d)


def load_spec(path: str = "") -> dict:
    """Task rules from the spec JSON (defaults to the installed one)."""
    if not path:
        share = Path(get_package_share_directory("frida_constants"))
        path = share / "data" / "tasks" / "ppc.json"
    return json.loads(Path(path).read_text())


class PickAndPlace:
    """Task tree; knows nothing about the venue beyond the spec."""

    PHASES = ("start", "cleanup", "breakfast")

    def __init__(self, sm, spec: dict, state: PPCState, node, clock=time.time):
        self.sm = sm
        self.spec = spec
        self.state = state
        self.node = node
        self.clock = clock
        if not state.breakfast:
            state.breakfast = [Item(b["object"]) for b in spec["breakfast"]]

    def elapsed(self) -> float:
        return 0.0 if self.state.started_at is None else self.clock() - self.state.started_at

    def say(self, text: str) -> Status:
        self.sm.hri.say(text, wait=False)
        return OK

    def destination(self, name: str) -> str:
        dest = self.spec["destinations"]
        return dest.get(self.sm.hri.deterministic_categorization(name), dest["*"])

    def breakfast_rule(self, name: str) -> dict:
        return next(b for b in self.spec["breakfast"] if b["object"] == name)

    @staticmethod
    def find(items: list, name: str):
        return next((i for i in items if i.name == name), None)

    # ── Leaves ──

    def wait_start(self) -> Status:
        self.say("Waiting for the start button.")
        while not (self.sm.hri.start_button_clicked or self.sm.hri.mock_data):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        while self.sm.nav.check_door()[0] != OK:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.state.started_at = self.clock()
        return OK

    def go(self, place: str) -> Status:
        if self.state.location == place:
            return OK
        dock = place not in self.spec["trash"]
        status = self.sm.composite.navigate_and_dock(place, dock=dock)
        self.state.location = place if status == OK else None
        return status

    def perceive_table(self) -> Status:
        self.go(self.spec["table"])
        status, dets = self.sm.composite.detect_on_surface()
        self.state.objects = [Item(d.classname) for d in dets or []]
        self.sm.composite.announce_objects([o.name for o in self.state.objects])
        return status

    def free_gripper(self) -> Status:
        status = self.sm.composite.ensure_gripper_empty(self.state.holding)
        if status == OK:
            self.state.holding = None
        return status

    def pick(self, item: Item, source: str) -> Status:
        self.say(f"I will pick the {item.name}.")
        if source in self.spec["shelves"]:
            heights = self.sm.composite.shelf_heights()
            status = self.sm.composite.pick_from_shelf(item.name, heights)
        else:
            status = self.sm.composite.pick_confirmed(item.name)
        if status == OK:
            item.status = "picked"
            self.state.holding = item.name
        return status

    def place(self, item: Item, place: str, next_to: str = "") -> Status:
        self.say(f"Placing the {item.name}.")
        if place in self.spec["shelves"]:
            status = self.sm.composite.place_on_shelf_near_similar(item.name)
        elif place in self.spec["trash"]:
            status = self.sm.manipulation.place(is_trash=True)
        else:
            status = self.sm.manipulation.place(close_to=next_to)
        if status == OK:
            item.status = "placed"
            self.state.holding = None
            self.sm.hri.publish_display_capture(f"Placed {item.name} at {place}")
        return status

    def pour(self, item: Item, container: str) -> Status:
        self.say(f"Pouring the {item.name}.")
        return self.sm.manipulation.pour(item.name, container, object_already_grasped=True)

    # ── Loops ──

    def next_object(self):
        if self.elapsed() > self.spec["cleanup_until_s"]:
            return None
        return next((o for o in self.state.objects if o.status == "pending"), None)

    def next_breakfast(self):
        return next((b for b in self.state.breakfast if b.status == "pending"), None)

    @staticmethod
    def done(item: Item, ok: bool):
        if item.status == "pending":
            item.status = "skipped"

    def clean(self, obj: Item):
        table, dest = self.spec["table"], self.destination(obj.name)
        return sequence(
            f"clean_{obj.name}",
            Action("navigate_to_table", lambda: self.go(table)),
            Action("ensure_gripper_empty", self.free_gripper),
            Action("pick_object", lambda: self.pick(obj, table)),
            Action("navigate_to_placement", lambda: self.go(dest)),
            Action("place_object", lambda: self.place(obj, dest)),
        )

    def serve(self, item: Item):
        rule, table = self.breakfast_rule(item.name), self.spec["table"]
        container = self.find(self.state.breakfast, rule.get("pour_into", ""))
        anchor = self.find(self.state.breakfast, rule.get("next_to", ""))
        next_to = anchor.name if anchor and anchor.status == "placed" else ""
        steps = [
            Action("navigate_to_item_source", lambda: self.go(rule["from"])),
            Action("ensure_gripper_empty", self.free_gripper),
            Action("pick_breakfast_item", lambda: self.pick(item, rule["from"])),
            Action("navigate_to_dining", lambda: self.go(table)),
        ]
        if container and container.status == "placed":
            steps.append(
                optional(Action("pour_into_bowl", lambda: self.pour(item, container.name)))
            )
        steps.append(Action("place_breakfast_item", lambda: self.place(item, table, next_to)))
        return sequence(f"breakfast_{item.name}", *steps)

    def build_tree(self, start_at: str = "start"):
        phases = {
            "start": [Action("wait_start", self.wait_start)],
            "cleanup": [
                optional(Action("perceive_table", self.perceive_table)),
                ForEach("cleanup_loop", self.next_object, self.clean, self.done),
            ],
            "breakfast": [ForEach("breakfast", self.next_breakfast, self.serve, self.done)],
        }
        if start_at not in phases:
            raise ValueError(f"start_at must be one of {self.PHASES}")
        children = []
        for phase in self.PHASES[self.PHASES.index(start_at) :]:
            children += phases[phase]
        return sequence("pick_and_place", *children)

    def out_of_time(self) -> bool:
        return self.elapsed() > self.spec["time_limit_s"]


class PickAndPlaceTM(Node):
    """ROS entry: params, subtask manager and the tree runner."""

    def __init__(self):
        super().__init__("pickandplace_task_manager")
        mock = self.declare_parameter("mock_areas", "").value
        self.start_at = self.declare_parameter("start_at", "start").value
        state_file = self.declare_parameter("state_file", "").value
        spec_file = self.declare_parameter("spec_file", "").value
        self.step = self.declare_parameter("step", False).value
        self.log_dir = self.declare_parameter("log_dir", "~/.ros/ppc_runs").value

        from task_manager.utils.subtask_manager import SubtaskManager

        mock_areas = [a.strip() for a in mock.split(",") if a.strip()]
        self.sm = SubtaskManager(self, task=Task.PICK_AND_PLACE, mock_areas=mock_areas)
        self.sm.composite.enable_tf()
        state = PPCState()
        if state_file:
            state = PPCState.from_dict(json.loads(Path(state_file).read_text()))
        self.ppc = PickAndPlace(self.sm, load_spec(spec_file), state, self)
        CLog.fsm(self, "STATE", f"PPC ready (start_at={self.start_at}, mock={mock_areas}).")

    def run(self):
        ppc = self.ppc
        if self.start_at != "start" and ppc.state.started_at is None:
            ppc.state.started_at = ppc.clock()
        log = RunLog(
            self.log_dir,
            snapshot=lambda: asdict(ppc.state),
            on_leaf=self.sm.hri.publish_display_step,
            step=self.step,
        )
        runner = TreeRunner(ppc.build_tree(self.start_at), log, ppc.out_of_time)
        runner.run(spin=lambda: rclpy.spin_once(self, timeout_sec=0.0))
        self.sm.hri.publish_display_step("end")
        self.sm.manipulation.move_to_position("nav_pose")
        CLog.fsm(self, "TIMER", f"Total {ppc.elapsed():.1f}s — log in {log.dir}")
        for name, seconds in log.report():
            CLog.fsm(self, "TIMER", f"{name}: {seconds:.1f}s")


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceTM()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
