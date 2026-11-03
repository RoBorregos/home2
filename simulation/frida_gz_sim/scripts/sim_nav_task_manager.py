#!/usr/bin/env python3
"""Sim navigation task manager: drive a route of areas with the real NavigationTasks.

Exercises task_manager's move_to_location() -> /navigation/go_to_map_area, and checks
each result against the robot's ground-truth Gazebo pose instead of trusting the report.
"""

import json
import math
import subprocess
import time

import rclpy
from frida_gz_sim.nav import MAP_NAME, NAV_ROUTE
from rclpy.node import Node
from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.utils.status import Status
from task_manager.utils.task import Task

# A goal counts as reached when the true pose is this close to the areas-file pose
POSITION_TOLERANCE = 0.45
HEADING_TOLERANCE = 25.0


def gazebo_pose(model: str = "frida"):
    """Ground-truth (x, y, yaw) of the robot straight from Gazebo."""
    try:
        out = subprocess.run(
            ["gz", "model", "-m", model, "-p"],
            capture_output=True,
            text=True,
            timeout=15,
        ).stdout
    except (subprocess.SubprocessError, FileNotFoundError):
        return None
    position, orientation = None, None
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("[") and line.endswith("]"):
            values = [float(v) for v in line[1:-1].split()]
            if position is None and len(values) == 3:
                position = values
            elif orientation is None and len(values) == 3:
                orientation = values
    if position is None or orientation is None:
        return None
    return position[0], position[1], orientation[2]


class SimNavTM(Node):
    def __init__(self):
        super().__init__("sim_nav_task_manager")
        self.settle_time = self.declare_parameter("settle_time", 2.0).value
        route = self.declare_parameter("route", "").value
        self.route = (
            [tuple(pair.split("/")) for pair in route.split(",")]
            if route
            else list(NAV_ROUTE)
        )
        self.navigation = NavigationTasks(self, task=Task.DEBUG)
        self.areas = self._load_areas()
        self.results = []

    def _load_areas(self) -> dict:
        status, areas = self.navigation.retrieve_areas()
        if status == Status.EXECUTION_SUCCESS and isinstance(areas, dict):
            self.get_logger().info(f"Areas from nav_central: {len(areas)} rooms")
            return areas
        from ament_index_python.packages import get_package_share_directory

        path = (
            f"{get_package_share_directory('map_context')}"
            f"/maps/areas/areas_{MAP_NAME}.json"
        )
        self.get_logger().warn(f"Falling back to {path}")
        with open(path) as f:
            return json.load(f)

    def wait(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def target_pose(self, location: str, sublocation: str):
        coords = self.areas.get(location, {}).get(sublocation)
        if coords is None:
            return None
        yaw = 2.0 * math.atan2(coords[5], coords[6])
        return coords[0], coords[1], yaw

    def check(self, location: str, sublocation: str) -> str:
        """Compare the robot's true pose with the pose the areas file asked for."""
        target = self.target_pose(location, sublocation)
        actual = gazebo_pose()
        if target is None or actual is None:
            return "no ground truth"
        distance = math.hypot(actual[0] - target[0], actual[1] - target[1])
        heading = abs(
            math.degrees(
                math.atan2(
                    math.sin(actual[2] - target[2]), math.cos(actual[2] - target[2])
                )
            )
        )
        verdict = (
            "ok"
            if distance < POSITION_TOLERANCE and heading < HEADING_TOLERANCE
            else "off target"
        )
        return f"{verdict} (error {distance:.2f} m, {heading:.1f} deg)"

    def run(self):
        self.get_logger().info(f"Sim navigation started: {self.route}")
        for location, sublocation in self.route:
            if self.target_pose(location, sublocation) is None:
                self.get_logger().error(
                    f"{location}/{sublocation} not in the areas file"
                )
                self.results.append((location, sublocation, "unknown area", ""))
                continue
            start = time.time()
            status, error = self.navigation.move_to_location(location, sublocation)
            elapsed = time.time() - start
            self.wait(self.settle_time)
            outcome = (
                "success" if status == Status.EXECUTION_SUCCESS else f"failed: {error}"
            )
            self.get_logger().info(
                f"{location}/{sublocation}: {outcome} in {elapsed:.0f}s - {self.check(location, sublocation)}"
            )
            self.results.append(
                (location, sublocation, outcome, self.check(location, sublocation))
            )
        self.report()

    def report(self):
        reached = sum(
            1
            for *_, outcome, verdict in self.results
            if outcome == "success" and verdict.startswith("ok")
        )
        self.get_logger().info("===== SIM NAVIGATION REPORT =====")
        for location, sublocation, outcome, verdict in self.results:
            self.get_logger().info(
                f"  {location}/{sublocation:16s} {outcome:24s} {verdict}"
            )
        self.get_logger().info(f"  reached {reached}/{len(self.results)} goal(s)")


def main(args=None):
    rclpy.init(args=args)
    node = SimNavTM()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
