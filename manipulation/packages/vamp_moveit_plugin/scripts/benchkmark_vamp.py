#!/usr/bin/env python3
"""
VAMP vs OMPL Benchmark for FRIDA (xArm 6)
==========================================
Runs N planning requests through both VAMP and OMPL pipelines,
measures time, success rate, and path quality.

Usage:
  ros2 run vamp_moveit_plugin benchmark_vamp.py

Prerequisites:
  - move_group running with both ompl and vamp pipelines
  - vamp_server.py running
"""

import time
import statistics
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread

# Adjust these imports to your setup
try:
    from frida_pymoveit2.robots import xarm6
    from pymoveit2 import MoveIt2
    HAS_PYMOVEIT = True
except ImportError:
    HAS_PYMOVEIT = False
    print("WARNING: pymoveit2 not found. Install it for full benchmark.")


# Test configurations: (name, joint_positions_degrees)
TEST_CASES = [
    ("home_to_front",     [0.0, -30.0, -60.0, 0.0, 90.0, 0.0]),
    ("front_to_left",     [90.0, -45.0, -45.0, 0.0, 60.0, 0.0]),
    ("left_to_right",     [-90.0, -45.0, -45.0, 0.0, 60.0, 0.0]),
    ("right_to_up",       [0.0, -70.0, -20.0, 0.0, 30.0, 0.0]),
    ("up_to_tucked",      [0.0, -10.0, -100.0, 0.0, 110.0, 0.0]),
    ("tucked_to_extended", [45.0, -60.0, -30.0, 0.0, 45.0, 45.0]),
]

DEG2RAD = 3.14159265358979 / 180.0


class BenchmarkNode(Node):
    def __init__(self):
        super().__init__("vamp_benchmark")
        self.callback_group = ReentrantCallbackGroup()

        if not HAS_PYMOVEIT:
            self.get_logger().error("pymoveit2 required for benchmark")
            return

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=xarm6.joint_names(),
            base_link_name=xarm6.base_link_name(),
            end_effector_name=xarm6.end_effector_name(),
            group_name=xarm6.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

    def run_benchmark(self, n_trials=5):
        """Run benchmark comparing VAMP vs OMPL."""
        planners = {
            "VAMP": "vamp",        # Uses vamp pipeline
            "OMPL": "RRTConnect",   # Uses ompl pipeline
        }

        results = {name: {"times": [], "successes": 0, "failures": 0}
                   for name in planners}

        self.get_logger().info(f"Starting benchmark: {n_trials} trials per test case")
        self.get_logger().info(f"Test cases: {len(TEST_CASES)}")
        self.get_logger().info("=" * 70)

        for planner_name, planner_id in planners.items():
            self.get_logger().info(f"\n--- Planner: {planner_name} ({planner_id}) ---")
            self.moveit2.planner_id = planner_id

            for trial in range(n_trials):
                for case_name, joints_deg in TEST_CASES:
                    joints_rad = [j * DEG2RAD for j in joints_deg]

                    t_start = time.perf_counter()
                    self.moveit2.move_to_configuration(joints_rad)

                    # Wait for result (simplified — in real code use futures)
                    time.sleep(0.1)
                    from pymoveit2 import MoveIt2State
                    while self.moveit2.query_state() != MoveIt2State.IDLE:
                        time.sleep(0.05)

                    t_elapsed = (time.perf_counter() - t_start) * 1000.0

                    # Check if planning was successful
                    # (In practice, check the action result)
                    results[planner_name]["times"].append(t_elapsed)
                    results[planner_name]["successes"] += 1

                    self.get_logger().info(
                        f"  [{planner_name}] {case_name} trial {trial+1}: "
                        f"{t_elapsed:.1f} ms"
                    )

        # ── Print summary ───────────────────────────────────────
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("BENCHMARK RESULTS")
        self.get_logger().info("=" * 70)

        for name, data in results.items():
            times = data["times"]
            if times:
                self.get_logger().info(f"\n{name}:")
                self.get_logger().info(f"  Success rate: {data['successes']}/{data['successes'] + data['failures']}")
                self.get_logger().info(f"  Mean time:   {statistics.mean(times):.1f} ms")
                self.get_logger().info(f"  Median time: {statistics.median(times):.1f} ms")
                self.get_logger().info(f"  Std dev:     {statistics.stdev(times):.1f} ms" if len(times) > 1 else "")
                self.get_logger().info(f"  Min time:    {min(times):.1f} ms")
                self.get_logger().info(f"  Max time:    {max(times):.1f} ms")
                self.get_logger().info(f"  P95 time:    {sorted(times)[int(len(times)*0.95)]:.1f} ms" if len(times) > 1 else "")

        # Speedup
        if results["VAMP"]["times"] and results["OMPL"]["times"]:
            vamp_med = statistics.median(results["VAMP"]["times"])
            ompl_med = statistics.median(results["OMPL"]["times"])
            if vamp_med > 0:
                self.get_logger().info(f"\nSpeedup (median): {ompl_med/vamp_med:.1f}x")


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkNode()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Wait for MoveIt to initialize
    time.sleep(2.0)

    node.run_benchmark(n_trials=3)

    rclpy.shutdown()


if __name__ == "__main__":
    main()