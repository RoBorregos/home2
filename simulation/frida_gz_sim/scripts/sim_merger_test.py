#!/usr/bin/env python3
"""Sim check for the GPSR merger with nav2 path distances (PR #1428).

Times /navigation/query_path_matrix for growing area counts, then merges sample
batches with straight-line vs path costs and compares their real route length.
"""

import math
import time
from types import SimpleNamespace

import rclpy
from frida_constants.navigation_constants import TIMEOUT_NAV_QUERY_MATRIX
from frida_gz_sim.nav import MAP_NAME
from rclpy.node import Node
from sim_nav_task_manager import gazebo_pose
from task_manager.gpsr.merger import merge
from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.utils.status import Status
from task_manager.utils.task import Task

# Mirrors gpsr_task_manager.DETOUR_FACTOR
DETOUR_FACTOR = 1.4

TIMING_AREAS = [
    ("kitchen", "dinner_table"),
    ("living_room", "sofa"),
    ("bedroom", "bed"),
    ("laundry", "washing_machine"),
    ("entrance", "safe_place"),
    ("kitchen", "refrigerator"),
    ("living_room", "coffee_table"),
    ("laundry", "table"),
    ("exit", "safe_place"),
    ("kitchen", "sink"),
]

SCENARIOS = {
    "deliver_plus_two_visits": [
        [
            ("go_to", "kitchen/dinner_table"),
            ("pick_object", "cup"),
            ("go_to", "living_room/sofa"),
            ("give_object", None),
        ],
        [("go_to", "bedroom/bed"), ("find_person_by_name", "Angel")],
        [("go_to", "laundry/washing_machine"), ("find_object", "towel")],
    ],
    "fridge_to_bed": [
        [("go_to", "entrance"), ("find_person_by_name", "Morgan")],
        [
            ("go_to", "kitchen/refrigerator"),
            ("pick_object", "apple"),
            ("go_to", "bedroom/bed"),
            ("place_object", None),
        ],
        [("go_to", "laundry/table"), ("count", "towels")],
    ],
    "three_rooms": [
        [("go_to", "living_room/coffee_table"), ("find_object", "remote")],
        [("go_to", "kitchen/sink"), ("find_object", "sponge")],
        [("go_to", "exit"), ("find_person_by_name", "Jules")],
    ],
}


def build(spec):
    cmds = []
    for actions in spec:
        objs = []
        for kind, arg in actions:
            fields = {"location_to_go": arg} if kind == "go_to" else {"arg": arg}
            objs.append(SimpleNamespace(action=kind, **fields))
        cmds.append(SimpleNamespace(commands=objs))
    return cmds


def to_area(name):
    if not name:
        return None
    loc, _, sub = name.partition("/")
    return (loc, sub or "safe_place")


class SimMergerTest(Node):
    def __init__(self):
        super().__init__("sim_merger_test")
        sizes = self.declare_parameter("sizes", "3,5,8,10").value
        self.sizes = [int(s) for s in sizes.split(",") if s]
        self.drive = self.declare_parameter("drive", False).value
        self.nav = NavigationTasks(self, task=Task.DEBUG)
        self.areas = self._load_areas()

    def _load_areas(self):
        status, areas = self.nav.retrieve_areas()
        if status == Status.EXECUTION_SUCCESS and isinstance(areas, dict):
            return areas
        import json

        from ament_index_python.packages import get_package_share_directory

        path = f"{get_package_share_directory('map_context')}/maps/areas/areas_{MAP_NAME}.json"
        with open(path) as f:
            return json.load(f)

    def xy(self, area):
        pose = self.areas.get(area[0], {}).get(area[1]) if area else None
        return (float(pose[0]), float(pose[1])) if pose else None

    def matrix(self, areas):
        t0 = time.monotonic()
        status, m = self.nav.get_path_matrix(areas)
        elapsed = time.monotonic() - t0
        if status != Status.EXECUTION_SUCCESS:
            self.get_logger().error(f"matrix failed: {m}")
            return {}, elapsed
        return m, elapsed

    def run_timing(self):
        rows = []
        for n in self.sizes:
            areas = TIMING_AREAS[:n]
            m, elapsed = self.matrix(areas)
            total = (n + 1) * n - n
            solved = sum(1 for (s, g) in m if s != g)
            rows.append((n, elapsed, solved, total))
            self.get_logger().info(
                f"matrix n={n}: {elapsed:.1f}s, {solved}/{total} pairs"
            )
        self.get_logger().info(
            f"--- matrix timing (budget {TIMEOUT_NAV_QUERY_MATRIX:.0f}s) ---"
        )
        for n, elapsed, solved, total in rows:
            flag = "" if solved == total else "  <- incomplete"
            self.get_logger().info(f"n={n:2d}  {elapsed:5.1f}s  {solved}/{total}{flag}")

    def route_cost(self, plan, matrix, origin_xy):
        """Real meters for the plan's go_to legs; unsolved legs use straight line."""
        total, prev, missing = 0.0, None, 0
        for pa in plan.actions:
            if getattr(pa.action, "action", "") != "go_to":
                continue
            dst = to_area(pa.location)
            if dst == prev:
                continue
            if (prev, dst) in matrix:
                total += matrix[(prev, dst)]
            else:
                missing += 1
                a = origin_xy if prev is None else self.xy(prev)
                total += math.dist(a, self.xy(dst))
            prev = dst
        return total, missing

    def go_tos(self, plan):
        return [
            pa.location
            for pa in plan.actions
            if getattr(pa.action, "action", "") == "go_to"
        ]

    def run_scenario(self, name, spec):
        cmds = build(spec)
        pose = gazebo_pose()
        origin = (pose[0], pose[1]) if pose else None
        areas = []
        for cmd in cmds:
            for a in cmd.commands:
                area = to_area(getattr(a, "location_to_go", None))
                if area and area not in areas:
                    areas.append(area)
        m, elapsed = self.matrix(areas)

        def path_cost(src_name, dst_name):
            dst, src = to_area(dst_name), to_area(src_name)
            if src == dst:
                return 0.0
            if (src, dst) in m:
                return m[(src, dst)]
            a = origin if src is None else self.xy(src)
            b = self.xy(dst)
            return None if a is None or b is None else DETOUR_FACTOR * math.dist(a, b)

        euclid = merge(cmds, locator=lambda n: self.xy(to_area(n)), origin=origin)
        path = merge(cmds, origin=origin, distance=path_cost)
        e_cost, e_miss = self.route_cost(euclid, m, origin)
        p_cost, p_miss = self.route_cost(path, m, origin)
        log = self.get_logger().info
        log(f"--- {name}: {len(areas)} areas, matrix {elapsed:.1f}s ---")
        log(
            f"euclid order: {self.go_tos(euclid)}  real {e_cost:.2f} m (unsolved legs {e_miss})"
        )
        log(
            f"path   order: {self.go_tos(path)}  real {p_cost:.2f} m (unsolved legs {p_miss})"
        )
        saved = e_cost - p_cost
        log(
            f"path merger saves {saved:.2f} m"
            if saved > 0.01
            else "same or worse route"
        )
        if p_cost > e_cost + 0.01:
            self.get_logger().warn("path merger chose a longer real route")
        return euclid, path

    def drive_plan(self, label, plan):
        self.nav.move_to_location("start_location", "safe_place")
        t0 = time.monotonic()
        for name in self.go_tos(plan):
            area = to_area(name)
            status, err = self.nav.move_to_location(*area)
            if status != Status.EXECUTION_SUCCESS:
                self.get_logger().error(f"{label}: {name} failed: {err}")
        self.get_logger().info(f"{label} drive time {time.monotonic() - t0:.0f}s")

    def run(self):
        self.run_timing()
        for name, spec in SCENARIOS.items():
            euclid, path = self.run_scenario(name, spec)
            if self.drive and self.go_tos(euclid) != self.go_tos(path):
                self.drive_plan(f"{name} euclid", euclid)
                self.drive_plan(f"{name} path", path)


def main():
    rclpy.init()
    node = SimMergerTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
