#!/usr/bin/env python3
"""Plans joint paths with VAMP for the vamp_moveit_plugin MoveIt planner."""

import math
import os
import sys
import time
from typing import Sequence

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vamp_moveit_plugin.srv import VampPlan

# VAMP's Python bindings are built inside its submodule and are not installed by colcon
sys.path.append(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../vamp/src")
)
import vamp  # noqa: E402

SERVICE_NAME = "/vamp_server/plan"


def xyz(point: Point) -> list[float]:
    return [point.x, point.y, point.z]


def roll_pitch_yaw(q: Quaternion) -> list[float]:
    """Euler angles in the R = Rz(yaw) Ry(pitch) Rx(roll) convention VAMP's shape constructors use."""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x**2 + q.y**2))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
    return [roll, pitch, yaw]


class PlanningError(Exception):
    """A request VAMP cannot plan; the plugin falls back to OMPL."""


class VampServer(Node):
    """Serves VampPlan requests by mirroring the MoveIt scene in VAMP and running RRT-Connect."""

    def __init__(self):
        super().__init__("vamp_server")
        robot_name = self.declare_parameter("robot", "frida_real").value
        self.collision_margin = self.declare_parameter("collision_margin", 0.01).value
        self.model_tolerance = self.declare_parameter("model_tolerance", 0.01).value

        self.robot = getattr(vamp, robot_name)
        self.joint_names = list(self.robot.joint_names())
        self.rng = self.robot.halton()
        self.rrtc_settings = vamp.RRTCSettings()
        self.rrtc_settings.range = self.declare_parameter("range", 2.0).value
        self.rrtc_settings.max_iterations = self.declare_parameter(
            "max_iterations", 100000
        ).value
        self.simplify_settings = vamp.SimplifySettings()

        self.create_service(VampPlan, SERVICE_NAME, self.plan_callback)
        self.get_logger().info(
            f"VAMP server ready for '{robot_name}' ({self.robot.n_spheres()} spheres)"
        )

    def plan_callback(
        self, request: VampPlan.Request, response: VampPlan.Response
    ) -> VampPlan.Response:
        start_time = time.perf_counter()
        try:
            path = self.plan(request)
        except PlanningError as error:
            response.success = False
            response.message = str(error)
            self.get_logger().warn(f"No path: {error}")
            return response

        response.success = True
        response.trajectory = self.to_trajectory(path)
        self.get_logger().info(
            f"Path with {len(path)} waypoints in "
            f"{(time.perf_counter() - start_time) * 1000.0:.1f} ms"
        )
        return response

    def plan(self, request: VampPlan.Request) -> np.ndarray:
        """Returns the simplified path as an (n, dof) array in VAMP's joint order."""
        start = self.to_vamp_order(request.joint_names, request.start)
        goal = self.to_vamp_order(request.joint_names, request.goal)
        self.check_model(start, request.start_end_effector_pose)

        environment = self.build_environment(request, start)
        if not self.robot.validate(start, environment):
            raise PlanningError("start state collides in the sphere model")
        if not self.robot.validate(goal, environment):
            raise PlanningError("goal state collides in the sphere model")

        result = self.robot.rrtc(start, goal, environment, self.rrtc_settings, self.rng)
        if not result.solved:
            raise PlanningError(
                f"RRT-Connect failed after {result.iterations} iterations"
            )
        simplified = self.robot.simplify(
            result.path, environment, self.simplify_settings, self.rng
        )
        return simplified.path.numpy()

    def to_vamp_order(
        self, names: Sequence[str], values: Sequence[float]
    ) -> np.ndarray:
        by_name = dict(zip(names, values))
        missing = [name for name in self.joint_names if name not in by_name]
        if missing:
            raise PlanningError(f"request lacks joints {missing}")
        return np.array([by_name[name] for name in self.joint_names], dtype=np.float32)

    def check_model(self, start: np.ndarray, end_effector_pose: Pose):
        """Warns when VAMP's forward kinematics disagree with MoveIt, i.e. the sphere model is out of date."""
        position = np.asarray(self.robot.eefk(start), dtype=float)[:3, 3]
        error = np.linalg.norm(position - xyz(end_effector_pose.position))
        if error > self.model_tolerance:
            self.get_logger().error(
                f"VAMP end effector is {error * 100.0:.1f} cm away from MoveIt's: "
                "regenerate the collision model for the current URDF",
                once=True,
            )

    def build_environment(
        self, request: VampPlan.Request, start: np.ndarray
    ) -> vamp.Environment:
        margin = self.collision_margin
        environment = vamp.Environment()

        for center, radius in zip(request.sphere_centers, request.sphere_radii):
            environment.add_sphere(vamp.Sphere(xyz(center), radius + margin))

        for pose, size in zip(request.box_poses, request.box_sizes):
            half_extents = [
                size.x / 2 + margin,
                size.y / 2 + margin,
                size.z / 2 + margin,
            ]
            environment.add_cuboid(
                vamp.Cuboid(
                    xyz(pose.position), roll_pitch_yaw(pose.orientation), half_extents
                )
            )

        for pose, radius, length in zip(
            request.cylinder_poses, request.cylinder_radii, request.cylinder_lengths
        ):
            # The endpoint constructor of vamp.Cylinder writes to uninitialized memory, so use the center one
            environment.add_capsule(
                vamp.Cylinder(
                    xyz(pose.position),
                    roll_pitch_yaw(pose.orientation),
                    radius + margin,
                    length,
                )
            )

        attached_radii = list(request.attached_sphere_radii)
        if attached_radii:
            attachment = vamp.Attachment(np.identity(4))
            attachment.add_spheres(
                [
                    vamp.Sphere(xyz(center), radius)
                    for center, radius in zip(
                        request.attached_sphere_centers, attached_radii
                    )
                ]
            )
            environment.attach(attachment)

        if request.point_cloud:
            point_radius = request.point_radius + margin
            # Points the robot overlaps at start are its own surface, which MoveIt's self filter also drops
            points = self.robot.filter_self_from_pointcloud(
                [xyz(point) for point in request.point_cloud],
                point_radius,
                start,
                vamp.Environment(),
            )
            min_radius, max_radius = self.robot.min_max_radii()
            environment.add_pointcloud(
                points,
                min_radius,
                max([max_radius, *attached_radii]),
                point_radius,
            )
        return environment

    def to_trajectory(self, path: np.ndarray) -> JointTrajectory:
        trajectory = JointTrajectory(joint_names=self.joint_names)
        trajectory.points = [
            JointTrajectoryPoint(
                positions=[float(value) for value in waypoint],
                time_from_start=Duration(),
            )
            for waypoint in path
        ]
        return trajectory


def main(args=None):
    rclpy.init(args=args)
    node = VampServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
