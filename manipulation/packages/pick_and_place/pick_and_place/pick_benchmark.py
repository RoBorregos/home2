#!/usr/bin/env python3

"""Pick benchmark: run N trials per object and record success/retention rate.

Cutlery mode picks spoon/fork/knife from the table; shelf mode scans the
levels once then picks the given objects (scan_environment=True). The operator
resets the object and confirms each outcome, since the gripper bit is not a
reliable retention signal. Results are printed and written to a CSV.

Usage (inside the manipulation container, with the manip launch running):
    python3 pick_benchmark.py --mode cutlery --trials 10
    python3 pick_benchmark.py --mode shelf --objects blue_cereal_box --trials 10
    python3 pick_benchmark.py --mode object --objects cup bowl --trials 10
"""

import argparse
import csv
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from frida_interfaces.action import ManipulationAction
from frida_interfaces.msg import ManipulationTask
from frida_interfaces.srv import GetOptimalPositionForPlane
from frida_constants.manipulation_constants import MANIPULATION_ACTION_SERVER

# Shelf level surface heights in base_link Z (match pickandplace_task_manager).
SHELF_LEVEL_HEIGHTS = [0.475, 0.827, 1.201]
SHELF_SCAN_TOLERANCE = 0.1


class PickBenchmark(Node):
    def __init__(self):
        super().__init__("pick_benchmark")
        self._action_client = ActionClient(
            self, ManipulationAction, MANIPULATION_ACTION_SERVER
        )
        self._optimal_client = self.create_client(
            GetOptimalPositionForPlane, "/manipulation/get_optimal_position_for_plane"
        )

    def scan_shelf_levels(self):
        """Face each shelf level to build the octomap before shelf picks."""
        if not self._optimal_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("get_optimal_position_for_plane not available!")
            return
        for height in SHELF_LEVEL_HEIGHTS:
            req = GetOptimalPositionForPlane.Request()
            req.plane_est_min_height = height - SHELF_SCAN_TOLERANCE
            req.plane_est_max_height = height + SHELF_SCAN_TOLERANCE
            req.table_or_shelf = False
            req.approach_plane = True
            self.get_logger().info(f"Scanning shelf level at {height:.3f} m")
            future = self._optimal_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            time.sleep(2.0)

    def send_pick(self, object_name, scan_environment):
        """Send one pick goal; return (action_success, grasp_score, duration_s)."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("manipulation action server not available!")
            return False, 0.0, 0.0
        goal = ManipulationAction.Goal()
        goal.task_type = ManipulationTask.PICK
        goal.pick_params.object_name = object_name
        goal.scan_environment = scan_environment
        t0 = time.time()
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, 0.0, time.time() - t0
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=180.0)
        wrapper = result_future.result()
        result = wrapper.result if wrapper else None
        if result is None:
            return False, 0.0, time.time() - t0
        score = float(getattr(result, "grasp_score", 0.0) or 0.0)
        return bool(result.success), score, time.time() - t0

    def run_trials(self, suite, object_name, trials, scan_environment, writer):
        """Run N trials, recording action result and operator-confirmed success."""
        ok = 0
        for i in range(1, trials + 1):
            input(
                f"\n[{suite}] trial {i}/{trials}: place '{object_name}', ENTER (estop ready)..."
            )
            success, score, dur = self.send_pick(object_name, scan_environment)
            retained = (
                input("  grasped AND held on lift? (y/n): ")
                .strip()
                .lower()
                .startswith("y")
            )
            ok += 1 if retained else 0
            writer.writerow(
                [
                    suite,
                    object_name,
                    i,
                    int(success),
                    round(score, 3),
                    int(retained),
                    round(dur, 1),
                ]
            )
            print(
                f"  -> action_success={success} score={score:.2f} retained={retained}"
            )
        rate = 100.0 * ok / max(1, trials)
        print(f"[{suite}] success: {ok}/{trials} = {rate:.0f}%")
        return ok, trials


def main(args=None):
    parser = argparse.ArgumentParser(description="Pick benchmark")
    parser.add_argument(
        "--mode", choices=["cutlery", "shelf", "object"], default="cutlery"
    )
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument(
        "--objects", nargs="*", default=None, help="override object list"
    )
    parser.add_argument("--out", default="/tmp/pick_benchmark.csv")
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = PickBenchmark()
    summary = {}
    with open(parsed.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "suite",
                "object",
                "trial",
                "action_success",
                "grasp_score",
                "retained",
                "duration_s",
            ]
        )
        try:
            if parsed.mode == "shelf":
                objects = parsed.objects or ["blue_cereal_box"]
                input("Place the shelf objects, ENTER to scan levels (estop ready)...")
                node.scan_shelf_levels()
            else:
                objects = parsed.objects or (
                    ["spoon", "fork", "knife"] if parsed.mode == "cutlery" else ["cup"]
                )
            scan_env = parsed.mode == "shelf"
            for obj in objects:
                ok, total = node.run_trials(
                    parsed.mode, obj, parsed.trials, scan_env, writer
                )
                summary[obj] = (ok, total)
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
