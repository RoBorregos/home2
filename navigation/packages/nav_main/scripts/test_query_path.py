#!/usr/bin/env python3
"""
Integration test for the /navigation/query_path service (NavQuery.srv).

Runs the real Nav_Central.query_path code against a fake ComputePathToPose
action server that returns a path of known length, so it can run on any
machine with ROS2 + frida_interfaces built — no robot, rtabmap or nav2
required.

Usage:
    source install/setup.bash
    python3 navigation/packages/nav_main/scripts/test_query_path.py
"""

import importlib.util
import math
import os
import sys
import threading
import time
import types

# rtabmap_msgs is only imported by nav_central for the GetMap service, which
# query_path never touches — stub it when not installed (dev machines)
try:
    import rtabmap_msgs.srv  # noqa: F401
except ImportError:
    pkg = types.ModuleType("rtabmap_msgs")
    srv = types.ModuleType("rtabmap_msgs.srv")
    srv.GetMap = type("GetMap", (), {})
    pkg.srv = srv
    sys.modules["rtabmap_msgs"] = pkg
    sys.modules["rtabmap_msgs.srv"] = srv

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from frida_constants.navigation_constants import (
    COMPUTE_PATH_ACTION_SERVER,
    NAV_QUERY_SERVICE,
)
from frida_interfaces.srv import NavQuery


def import_nav_central():
    """Import nav_central.py as a module from this script's directory."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "nav_central.py")
    spec = importlib.util.spec_from_file_location("nav_central", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Fake path returned by the planner: (0,0) -> (3,0) -> (3,4) = 3 + 4 = 7.0 m
FAKE_PATH_POINTS = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]
EXPECTED_DISTANCE = 7.0

AREAS = {
    "kitchen": {"safe_place": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
    "living_room": {"sofa": [3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
}


class FakePlanner(Node):
    """Stands in for nav2 planner_server's ComputePathToPose action."""

    def __init__(self):
        super().__init__("fake_planner_server")
        self.last_goal = None
        self._server = ActionServer(
            self, ComputePathToPose, COMPUTE_PATH_ACTION_SERVER, self.execute
        )

    def execute(self, goal_handle):
        self.last_goal = goal_handle.request
        result = ComputePathToPose.Result()
        result.path.header.frame_id = "map"
        for x, y in FAKE_PATH_POINTS:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            result.path.poses.append(pose)
        goal_handle.succeed()
        return result


def build_nav_central(nav_central_module):
    """Instantiate the real Nav_Central, skipping the hardware setup."""
    node = nav_central_module.Nav_Central("nav_central_under_test")
    # Disarm the setup timer before anything spins — no robot to wait for
    node.destroy_timer(node._setup_timer)
    node._setup_done = True
    node.rtabmap_loaded = True
    node.nav2_paused = False
    node.nodes_status = True
    node.areas_data = AREAS
    # Slam pause/resume talk to rtabmap services that don't exist here;
    # they time out gracefully but slow the test down — skip them
    node.resume_slam = lambda: None
    node.pause_slam = lambda: None
    return node


def call_query(client, node, loc_a, sub_a, loc_b, sub_b, timeout=15.0):
    req = NavQuery.Request()
    req.location_a = loc_a
    req.sublocation_a = sub_a
    req.location_b = loc_b
    req.sublocation_b = sub_b
    future = client.call_async(req)
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)
    if not future.done():
        raise TimeoutError("query_path service call timed out")
    return future.result()


def main():
    rclpy.init()
    nav_central_module = import_nav_central()

    planner = FakePlanner()
    nav_central = build_nav_central(nav_central_module)
    client_node = Node("query_path_test_client")
    client = client_node.create_client(NavQuery, NAV_QUERY_SERVICE)

    executor = MultiThreadedExecutor(num_threads=6)
    for n in (planner, nav_central, client_node):
        executor.add_node(n)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    assert client.wait_for_service(timeout_sec=5.0), "query_path service never appeared"

    failures = []

    def check(name, condition, detail=""):
        if condition:
            print(f"  PASS  {name}")
        else:
            print(f"  FAIL  {name} {detail}")
            failures.append(name)

    print("\n[1] Known areas A -> B (kitchen -> living_room/sofa)")
    res = call_query(client, client_node, "kitchen", "safe_place", "living_room", "sofa")
    check("success is True", res.success, f"(error: {res.error})")
    check(
        f"distance == {EXPECTED_DISTANCE}",
        math.isclose(res.distance_meters, EXPECTED_DISTANCE, abs_tol=1e-6),
        f"(got {res.distance_meters})",
    )
    goal = planner.last_goal
    check("planner received use_start=True", goal is not None and goal.use_start)
    check(
        "planner received start = kitchen coords",
        goal is not None
        and math.isclose(goal.start.pose.position.x, 0.0)
        and math.isclose(goal.start.pose.position.y, 0.0),
    )
    check(
        "planner received goal = sofa coords",
        goal is not None
        and math.isclose(goal.goal.pose.position.x, 3.0)
        and math.isclose(goal.goal.pose.position.y, 4.0),
    )

    print("\n[2] Empty origin -> robot current pose (use_start must be False)")
    res = call_query(client, client_node, "", "", "living_room", "sofa")
    check("success is True", res.success, f"(error: {res.error})")
    check("planner received use_start=False", not planner.last_goal.use_start)

    print("\n[3] Unknown destination area")
    res = call_query(client, client_node, "", "", "garage", "car")
    check("success is False", not res.success)
    check("error mentions the area", "garage" in res.error, f"(got: {res.error})")

    print("\n[4] Unknown origin area")
    res = call_query(client, client_node, "basement", "stairs", "living_room", "sofa")
    check("success is False", not res.success)
    check("error mentions the area", "basement" in res.error, f"(got: {res.error})")

    print()
    if failures:
        print(f"{len(failures)} test(s) FAILED: {failures}")
    else:
        print("All query_path tests passed!")

    executor.shutdown()
    rclpy.shutdown()
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
