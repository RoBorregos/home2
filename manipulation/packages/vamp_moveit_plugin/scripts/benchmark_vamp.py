#!/usr/bin/env python3
"""Compares the "vamp" and "ompl" MoveIt pipelines on the same joint goals and scenes, planning only.

Needs move_group with both pipelines and the VAMP server running, e.g. the sim started with --vamp:
    ros2 run vamp_moveit_plugin benchmark_vamp.py --ros-args -p trials:=5 -p csv:=/tmp/vamp_benchmark.csv
Fallbacks are the plugin's "Falling back to OMPL" warnings read from /rosout.
"""

import csv
import math
import statistics
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from rcl_interfaces.msg import Log
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

GROUP = "xarm6"
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
PIPELINES = ("ompl", "vamp")
OBSTACLE_ID = "benchmark_obstacle"

# Each goal starts from the previous one, so the arm never has to move
GOALS_DEG = {
    "front": [0.0, -30.0, -60.0, 0.0, 90.0, 0.0],
    "left": [90.0, -45.0, -45.0, 0.0, 60.0, 0.0],
    "right": [-90.0, -45.0, -45.0, 0.0, 60.0, 0.0],
    "up": [0.0, -70.0, -20.0, 0.0, 30.0, 0.0],
    "tucked": [0.0, -10.0, -100.0, 0.0, 110.0, 0.0],
    "extended": [45.0, -60.0, -30.0, 0.0, 45.0, 45.0],
}

# Extra obstacle per scene in base_link: box size, position and yaw (rad); None keeps the sim scene as is
SCENES = {
    "open": None,
    "wall": ([0.05, 0.5, 0.4], [0.35, 0.25, 0.85], math.pi / 4),
}


@dataclass
class PipelineStats:
    planning_ms: list[float] = field(default_factory=list)
    path_rad: list[float] = field(default_factory=list)
    failures: int = 0
    fallbacks: int = 0

    @property
    def total(self) -> int:
        return len(self.planning_ms) + self.failures


class VampBenchmark(Node):
    """Sends the same motion plan requests to each pipeline in each scene and summarizes the answers."""

    def __init__(self):
        super().__init__("vamp_benchmark")
        self.trials = self.declare_parameter("trials", 5).value
        self.planning_time = self.declare_parameter("planning_time", 1.0).value
        self.csv_path = self.declare_parameter("csv", "").value
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.rows = []
        self.fallback_warnings = 0
        self.create_subscription(Log, "/rosout", self.count_fallback, 100)

    def count_fallback(self, log: Log):
        self.fallback_warnings += log.msg == "Falling back to OMPL"

    def run(self) -> dict[tuple[str, str], PipelineStats]:
        for client in (self.plan_client, self.scene_client):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"move_group's {client.srv_name} is not available")
        goals = [[math.radians(v) for v in goal] for goal in GOALS_DEG.values()]
        stats = {}
        for scene, obstacle in SCENES.items():
            self.set_obstacle(obstacle)
            for pipeline in PIPELINES:
                stats[(scene, pipeline)] = PipelineStats()
            for trial in range(self.trials):
                for name, start, goal in zip(GOALS_DEG, goals[-1:] + goals[:-1], goals):
                    for pipeline in PIPELINES:
                        self.record(
                            stats[(scene, pipeline)],
                            scene,
                            pipeline,
                            trial,
                            name,
                            self.plan(pipeline, start, goal),
                        )
        self.set_obstacle(None)
        return stats

    def set_obstacle(self, obstacle):
        collision_object = CollisionObject(id=OBSTACLE_ID)
        collision_object.header.frame_id = "base_link"
        collision_object.operation = CollisionObject.REMOVE
        if obstacle:
            size, position, yaw = obstacle
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.z, pose.orientation.w = (
                math.sin(yaw / 2),
                math.cos(yaw / 2),
            )
            collision_object.operation = CollisionObject.ADD
            collision_object.primitives = [
                SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
            ]
            collision_object.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [collision_object]
        future = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future)

    def plan(self, pipeline: str, start: list[float], goal: list[float]):
        request = MotionPlanRequest()
        request.pipeline_id = pipeline
        request.group_name = GROUP
        request.allowed_planning_time = self.planning_time
        request.num_planning_attempts = 1
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        request.start_state.is_diff = True
        request.start_state.joint_state.name = JOINT_NAMES
        request.start_state.joint_state.position = start
        request.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=name,
                        position=position,
                        tolerance_above=1e-3,
                        tolerance_below=1e-3,
                        weight=1.0,
                    )
                    for name, position in zip(JOINT_NAMES, goal)
                ]
            )
        ]
        warnings = self.fallback_warnings
        future = self.plan_client.call_async(
            GetMotionPlan.Request(motion_plan_request=request)
        )
        rclpy.spin_until_future_complete(self, future)
        # Gives the plugin's warning time to arrive through /rosout
        rclpy.spin_once(self, timeout_sec=0.05)
        return future.result().motion_plan_response, self.fallback_warnings > warnings

    def record(self, stats: PipelineStats, scene, pipeline, trial, goal, result):
        response, fallback = result
        success = response.error_code.val == response.error_code.SUCCESS
        planning_ms = response.planning_time * 1000.0
        path_rad = math.nan
        if success:
            points = response.trajectory.joint_trajectory.points
            positions = np.array([point.positions for point in points])
            path_rad = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
            stats.planning_ms.append(planning_ms)
            stats.path_rad.append(path_rad)
            stats.fallbacks += fallback
        else:
            stats.failures += 1
        self.rows.append(
            [
                scene,
                pipeline,
                trial,
                goal,
                success,
                fallback,
                f"{planning_ms:.2f}",
                f"{path_rad:.3f}",
            ]
        )

    def report(self, stats: dict[tuple[str, str], PipelineStats]):
        for (scene, pipeline), result in stats.items():
            times = sorted(result.planning_ms) or [math.nan]
            paths = result.path_rad or [math.nan]
            self.get_logger().info(
                f"{scene:5} {pipeline:4}: {len(result.planning_ms)}/{result.total} solved, "
                f"{result.fallbacks} fallbacks, "
                f"median {statistics.median(times):.1f} ms, "
                f"p95 {times[int(0.95 * (len(times) - 1))]:.1f} ms, "
                f"max {times[-1]:.1f} ms, "
                f"path {statistics.median(paths):.2f} rad"
            )
        if self.csv_path:
            with open(self.csv_path, "w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(
                    [
                        "scene",
                        "pipeline",
                        "trial",
                        "goal",
                        "success",
                        "fallback",
                        "planning_ms",
                        "path_rad",
                    ]
                )
                writer.writerows(self.rows)
            self.get_logger().info(
                f"Wrote {len(self.rows)} requests to {self.csv_path}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = VampBenchmark()
    try:
        start = time.perf_counter()
        node.report(node.run())
        node.get_logger().info(f"Benchmark took {time.perf_counter() - start:.1f} s")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
