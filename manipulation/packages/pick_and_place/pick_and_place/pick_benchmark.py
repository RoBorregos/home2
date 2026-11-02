#!/usr/bin/env python3

"""Manipulation benchmark: trials go to an appended CSV, and an operator confirms each outcome."""

import argparse
import csv
import json
import os
import subprocess
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node

from frida_interfaces.action import ManipulationAction
from frida_interfaces.msg import ManipulationTask
from frida_interfaces.srv import GetOptimalPositionForPlane
from frida_constants.manipulation_constants import MANIPULATION_ACTION_SERVER
from xarm_utils.shelf_levels import (
    SHELF_SCAN_TOLERANCE,
    get_shelf_levels,
)

CSV_HEADER = [
    "timestamp",
    "git_sha",
    "planner",
    "method",
    "suite",
    "scene",
    "object",
    "trial",
    "action_success",
    "retained",
    "failure_source",
    "pick_failure",
    "pick_s",
    "place_request",
    "place_action_success",
    "placed_ok",
    "neighbour_moved",
    "place_s",
    "note",
]
FAILURES = {"p": "perception", "l": "planning", "e": "execution", "o": "other"}
SPATIAL_POSITIONS = ["front", "back", "left", "right", "close"]


class PickBenchmark(Node):
    def __init__(self):
        super().__init__("pick_benchmark")
        self.declare_parameter("arena", 1)
        self.arena = self.get_parameter("arena").value
        self._action_client = ActionClient(
            self, ManipulationAction, MANIPULATION_ACTION_SERVER
        )
        self._optimal_client = self.create_client(
            GetOptimalPositionForPlane, "/manipulation/get_optimal_position_for_plane"
        )
        self._params_client = self.create_client(
            SetParameters, "/manipulation_core/set_parameters"
        )

    def set_method(self, method):
        """Select the pick method on manipulation_core; raise if it does not apply."""
        if not self._params_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("manipulation_core parameters not available")
        value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=method)
        request = SetParameters.Request()
        request.parameters = [Parameter(name="pick_method", value=value)]
        future = self._params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.results[0].successful:
            raise RuntimeError("could not set pick_method")

    def scan_shelf_levels(self):
        """Face each shelf level to build the octomap before shelf picks."""
        if not self._optimal_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("get_optimal_position_for_plane not available!")
            return
        for height in get_shelf_levels(self.arena):
            req = GetOptimalPositionForPlane.Request()
            req.plane_est_min_height = height - SHELF_SCAN_TOLERANCE
            req.plane_est_max_height = height + SHELF_SCAN_TOLERANCE
            req.table_or_shelf = False
            req.approach_plane = True
            self.get_logger().info(f"Scanning shelf level at {height:.3f} m")
            future = self._optimal_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            time.sleep(2.0)

    def send_goal(self, goal):
        """Send one goal; return (action_success, duration_s)."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("manipulation action server not available!")
            return False, 0.0
        t0 = time.time()
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, time.time() - t0
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=180.0)
        wrapper = result_future.result()
        result = wrapper.result if wrapper else None
        return bool(result and result.success), time.time() - t0

    def pick(self, object_name, scan_environment):
        goal = ManipulationAction.Goal()
        goal.task_type = ManipulationTask.PICK
        goal.pick_params.object_name = object_name
        goal.scan_environment = scan_environment
        return self.send_goal(goal)

    def place(self, request, reference, spot=None):
        """Place on a free spot, relative to reference, or at a forced (x, y, z) spot."""
        goal = ManipulationAction.Goal()
        goal.task_type = ManipulationTask.PLACE
        if spot:
            forced = goal.place_params.forced_pose
            forced.header.frame_id = "base_link"
            forced.pose.position.x, forced.pose.position.y = spot[:2]
            forced.pose.position.z = spot[2]
        elif request:
            goal.place_params.special_request = json.dumps(
                {"request": "close_by", "object": reference, "position": request}
            )
        return self.send_goal(goal)


def ask_yes(prompt):
    return input(f"  {prompt} (y/n): ").strip().lower().startswith("y")


def ask_failure():
    while True:
        answer = input("  failure: p=perception l=planning e=execution o=other: ")
        if answer.strip().lower() in FAILURES:
            return FAILURES[answer.strip().lower()]


def git_sha():
    try:
        return subprocess.run(
            [
                "git",
                "-C",
                os.path.dirname(os.path.abspath(__file__)),
                "rev-parse",
                "HEAD",
            ],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()[:8]
    except (OSError, subprocess.CalledProcessError):
        return os.environ.get("EXPERIMENT_SHA", "unknown")


def run_trial(node, args, writer, suite, scene, obj, trial, place_request, reference):
    """Run one pick, plus a place when requested, and write its CSV row."""
    input(
        f"\n[{suite} {scene}] {obj} trial {trial}: scene ready, ENTER (estop ready)..."
    )
    picked, pick_s = node.pick(obj, suite == "shelf")
    retained = ask_yes("grasped AND held on lift?")
    failure = "" if retained else ask_failure()
    place_ok, placed, moved, place_s = "", "", "", ""
    if retained and args.place:
        place_ok, place_s = node.place(place_request, reference)
        placed = int(ask_yes("upright, stable and in the requested zone?"))
        moved = int(ask_yes("neighbour knocked or displaced?"))
        place_ok, place_s = int(place_ok), round(place_s, 1)
    writer.writerow(
        [
            time.strftime("%Y-%m-%dT%H:%M:%S"),
            args.sha,
            args.planner,
            args.method,
            suite,
            scene,
            obj,
            trial,
            int(picked),
            int(retained),
            "" if retained else ("operator" if picked else "stack"),
            failure,
            round(pick_s, 1),
            place_request,
            place_ok,
            placed,
            moved,
            place_s,
            input("  note (ENTER for none): ").strip(),
        ]
    )
    args.csv_file.flush()
    return retained


def main(args=None):
    parser = argparse.ArgumentParser(description="Manipulation benchmark")
    parser.add_argument(
        "--mode",
        choices=["flat", "shelf", "object", "clutter", "spatial"],
        default="flat",
    )
    parser.add_argument(
        "--method", choices=["auto", "geometric", "gpd"], default="auto"
    )
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--objects", nargs="*", default=None, help="object labels")
    parser.add_argument("--place", action="store_true", help="place after each pick")
    parser.add_argument("--scene-file", help="JSON from generate_scenes.py")
    parser.add_argument("--scene", type=int, help="scene id for clutter mode")
    parser.add_argument("--reference", default="", help="spatial reference object")
    parser.add_argument("--planner", default="ompl")
    parser.add_argument("--out", default="/workspace/src/bags/experiment.csv")
    parsed, ros_args = parser.parse_known_args()
    parsed.sha = git_sha()
    if parsed.mode == "clutter" and not (parsed.scene_file and parsed.scene):
        parser.error("clutter needs --scene-file and --scene")
    if parsed.mode == "spatial" and not parsed.reference:
        parser.error("spatial needs --reference")
    parsed.place = parsed.place or parsed.mode in ("clutter", "spatial")

    rclpy.init(args=ros_args)
    node = PickBenchmark()
    node.set_method(parsed.method)
    os.makedirs(os.path.dirname(os.path.abspath(parsed.out)), exist_ok=True)
    new_file = not os.path.exists(parsed.out) or os.path.getsize(parsed.out) == 0
    summary = {}
    with open(parsed.out, "a", newline="") as f:
        parsed.csv_file = f
        writer = csv.writer(f)
        if new_file:
            writer.writerow(CSV_HEADER)
        try:
            if parsed.mode == "clutter":
                with open(parsed.scene_file) as scenes:
                    scene = next(
                        s
                        for s in json.load(scenes)["scenes"]
                        if s["id"] == parsed.scene
                    )
                for obj in scene["objects"]:
                    held = run_trial(
                        node, parsed, writer, "clutter", scene["id"], obj, 1, "", ""
                    )
                    ok, total = summary.get(obj, (0, 0))
                    summary[obj] = (ok + int(held), total + 1)
            else:
                if parsed.mode == "shelf":
                    objects = parsed.objects or ["blue_cereal_box"]
                    input("Place the shelf objects, ENTER to scan (estop ready)...")
                    node.scan_shelf_levels()
                else:
                    objects = parsed.objects or (
                        ["spoon", "fork", "knife"] if parsed.mode == "flat" else ["cup"]
                    )
                for obj in objects:
                    ok = 0
                    for i in range(1, parsed.trials + 1):
                        request = ""
                        if parsed.mode == "spatial":
                            request = SPATIAL_POSITIONS[
                                (i - 1) % len(SPATIAL_POSITIONS)
                            ]
                        ok += int(
                            run_trial(
                                node,
                                parsed,
                                writer,
                                parsed.mode,
                                "",
                                obj,
                                i,
                                request,
                                parsed.reference,
                            )
                        )
                    summary[obj] = (ok, parsed.trials)
        except KeyboardInterrupt:
            pass

    print("\n=== SUMMARY ===")
    for obj, (ok, total) in summary.items():
        print(f"  {obj}: {ok}/{total} = {100.0 * ok / max(1, total):.0f}%")
    print(f"CSV: {parsed.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
