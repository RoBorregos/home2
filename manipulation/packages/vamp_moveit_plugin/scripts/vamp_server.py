#!/usr/bin/env python3
"""
VAMP Planning Server — Optimized for FRIDA (xArm 6)
====================================================
Key design decisions:
  - frida_real is a 6-DOF model (6 revolute arm joints only)
  - VAMP expects numpy float64 arrays (C++ doubles) to prevent Segfaults.
  - Gripper joints are NOT part of the VAMP collision model
  - Path processing INVERTED for extreme speed: Prune -> Smooth -> Validate
  - C++ Shortcutting is COMPLETELY REMOVED to guarantee geometric stability.
"""

import sys
import os
import time
import traceback

import rclpy
from rclpy.node import Node
from vamp_moveit_plugin.srv import VampPlan
import numpy as np

actual_dir = os.path.dirname(os.path.abspath(__file__))
ruta_vamp = os.path.abspath(os.path.join(actual_dir, "../../vamp/src"))
sys.path.append(ruta_vamp)
import vamp

ARM_DOF = 6


class VampServer(Node):
    def __init__(self):
        super().__init__("vamp_server")

        
        self.declare_parameter("max_iterations", 50000)
        self.declare_parameter("range", 0.05)
        self.declare_parameter("security_margin", 0.08)
        self.declare_parameter("validation_step_size", 0.05)
        self.declare_parameter("min_waypoint_distance", 0.08)
        self.declare_parameter("smoothing_window", 5)
        self.declare_parameter("smoothing_passes", 3)
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("retry_range_multiplier", 1.5)
        self.declare_parameter("retry_iterations_multiplier", 2.0)

        
        self.security_margin = self.get_parameter("security_margin").value
        self.validation_step = self.get_parameter("validation_step_size").value
        self.min_wp_dist = self.get_parameter("min_waypoint_distance").value
        self.smooth_window = self.get_parameter("smoothing_window").value
        self.smooth_passes = self.get_parameter("smoothing_passes").value
        self.max_retries = self.get_parameter("max_retries").value
        self.retry_range_mult = self.get_parameter("retry_range_multiplier").value
        self.rng = vamp.frida_real.xorshift()
        self.retry_iter_mult = self.get_parameter("retry_iterations_multiplier").value

        
        self.base_settings = vamp.RRTCSettings()
        self.base_settings.max_iterations = self.get_parameter("max_iterations").value
        self.base_settings.range = self.get_parameter("range").value

        self.srv = self.create_service(
            VampPlan, "plan_vamp_path", self.plan_callback
        )

        self.get_logger().info(
            f"VAMP server ready | max_iter={self.base_settings.max_iterations} "
            f"range={self.base_settings.range} margin={self.security_margin}"
        )

    class Timer:
        def __init__(self, name, logger):
            self.name = name
            self.logger = logger
            self.elapsed = 0.0

        def __enter__(self):
            self.start = time.perf_counter()
            return self

        def __exit__(self, *args):
            self.elapsed = (time.perf_counter() - self.start) * 1000.0
            self.logger.info(f"  ⏱  {self.name}: {self.elapsed:.2f} ms")

    @staticmethod
    def to_vamp(joint_values):
        return np.array(joint_values[:ARM_DOF], dtype=np.float64)

    def build_environment(self, request):
        env = vamp.Environment()
        n_spheres = 0
        n_boxes = 0

        centers = np.array(request.sphere_centers_flat, dtype=np.float64)
        radii = np.array(request.sphere_radii, dtype=np.float64)

        for i in range(len(radii)):
            idx = i * 3
            pos = centers[idx: idx + 3].tolist()
            r = float(radii[i]) + self.security_margin
            env.add_sphere(vamp.Sphere(pos, r))
            n_spheres += 1

        box_centers = np.array(request.box_centers_flat, dtype=np.float64)
        box_sizes = np.array(request.box_sizes_flat, dtype=np.float64)
        num_boxes = len(box_sizes) // 3

        for i in range(num_boxes):
            idx = i * 3
            cx, cy, cz = box_centers[idx: idx + 3]
            lx, ly, lz = box_sizes[idx: idx + 3]

            lx_s = float(lx) + 2 * self.security_margin
            ly_s = float(ly) + 2 * self.security_margin
            lz_s = float(lz) + 2 * self.security_margin

            center = [float(cx), float(cy), float(cz)]
            half_extents = [lx_s / 2.0, ly_s / 2.0, lz_s / 2.0]
            rotation = [0.0, 0.0, 0.0]

            try:
                env.add_cuboid(vamp.Cuboid(center, rotation, half_extents))
                n_boxes += 1
            except Exception as e:
                self.get_logger().warn(f"add_cuboid failed: {e}, using sphere fallback")
                self._approximate_box_with_spheres(
                    env, float(cx), float(cy), float(cz),
                    float(lx), float(ly), float(lz)
                )
                n_boxes += 1

        return env, n_spheres, n_boxes

    def _approximate_box_with_spheres(self, env, cx, cy, cz, lx, ly, lz):
        step = 0.04
        radius = 0.025 + self.security_margin
        for axis in range(3):
            dims = [lx, ly, lz]
            center = [cx, cy, cz]
            half = dims[axis] / 2.0
            other = [a for a in range(3) if a != axis]
            n1 = max(2, int(dims[other[0]] / step))
            n2 = max(2, int(dims[other[1]] / step))
            for sign in [-1, 1]:
                for u in np.linspace(-dims[other[0]] / 2, dims[other[0]] / 2, n1):
                    for v in np.linspace(-dims[other[1]] / 2, dims[other[1]] / 2, n2):
                        pos = list(center)
                        pos[axis] = center[axis] + sign * half
                        pos[other[0]] = center[other[0]] + u
                        pos[other[1]] = center[other[1]] + v
                        env.add_sphere(vamp.Sphere(pos, radius))

    def validate_states(self, start, goal, env):
        is_start_valid = vamp.frida_real.validate(start, env)
        is_goal_valid = vamp.frida_real.validate(goal, env)
        if is_start_valid and is_goal_valid:
            return None

        clean_env = vamp.Environment()
        start_clean = vamp.frida_real.validate(start, clean_env)
        goal_clean = vamp.frida_real.validate(goal, clean_env)

        msgs = []
        if not is_start_valid:
            cause = "self-collision/joint-limits" if not start_clean else "environment collision"
            msgs.append(f"Start INVALID ({cause})")
        if not is_goal_valid:
            cause = "self-collision/joint-limits" if not goal_clean else "environment collision"
            msgs.append(f"Goal INVALID ({cause})")
        return " | ".join(msgs)

    def init_rng(self):
        try:
            return vamp.frida_real.halton()
        except TypeError:
            pass
        try:
            return vamp.frida_real.xorshift()
        except Exception as e:
            self.get_logger().error(f"RNG init failed: {e}")
            return None

    def validate_and_densify_path(self, path_nodes, env):
        n_points = len(path_nodes)
        dense_path = []
        for i in range(n_points - 1):
            p1, p2 = path_nodes[i], path_nodes[i + 1]
            dist = np.linalg.norm(p1 - p2)
            n_steps = max(2, int(dist / self.validation_step))
            for t in np.linspace(0, 1, n_steps, endpoint=False):
                interp = np.array(p1 + (p2 - p1) * t, dtype=np.float64)
                if not vamp.frida_real.validate(interp, env):
                    self.get_logger().error(f"Collision at segment {i}, t={t:.3f}")
                    return dense_path, False
                dense_path.append(interp)
        dense_path.append(path_nodes[-1])
        return dense_path, True

    def downsample_path(self, dense_path):
        if len(dense_path) < 2:
            return dense_path
        clean = [dense_path[0]]
        for wp in dense_path[1:]:
            if np.linalg.norm(wp - clean[-1]) >= self.min_wp_dist:
                clean.append(wp)
        if np.linalg.norm(dense_path[-1] - clean[-1]) > 0.01:
            clean.append(dense_path[-1])
        return clean

    def apply_smoothing_filter(self, path_nodes):
        if len(path_nodes) < 3:
            return path_nodes
        smoothed = np.array(path_nodes, dtype=np.float64).copy()
        n = len(smoothed)
        w = self.smooth_window
        for _ in range(self.smooth_passes):
            new = smoothed.copy()
            for i in range(1, n - 1):
                lo = max(0, i - w // 2)
                hi = min(n, i + w // 2 + 1)
                new[i] = np.mean(smoothed[lo:hi], axis=0)
            smoothed = new
        return list(smoothed)

    
    
    
    def plan_callback(self, request, response):
        total_start = time.perf_counter()
        self.get_logger().info("=" * 60)
        self.get_logger().info("NEW PLANNING REQUEST")

        try:
            with self.Timer("Environment build", self.get_logger()):
                env, n_sph, n_box = self.build_environment(request)
            
            start = self.to_vamp(request.start_state)
            goal = self.to_vamp(request.goal_state)

            with self.Timer("State validation", self.get_logger()):
                diag = self.validate_states(start, goal, env)
            if diag:
                self.get_logger().error(diag)
                response.success = False
                return response

            rng = self.init_rng()
            if rng is None:
                response.success = False
                return response

            result = None
            settings = vamp.RRTCSettings()
            settings.max_iterations = self.base_settings.max_iterations
            settings.range = self.base_settings.range

            for attempt in range(self.max_retries):
                with self.Timer(
                    f"RRTConnect (attempt {attempt + 1}, "
                    f"iter={settings.max_iterations}, range={settings.range:.3f})",
                    self.get_logger(),
                ):
                    result = vamp.frida_real.rrtc(start, goal, env, settings, rng)

                if result and result.solved:
                    self.get_logger().info(f"  Solved on attempt {attempt + 1}")
                    break

                settings.max_iterations = int(settings.max_iterations * self.retry_iter_mult)
                settings.range *= self.retry_range_mult
                self.get_logger().warn(f"  Attempt {attempt + 1} failed, retrying...")

            if not result or not result.solved:
                self.get_logger().warn("All planning attempts failed.")
                response.success = False
                return response

            raw_path = result.path

            path_nodes = None
            with self.Timer("Path simplify (SHORTCUT+BSPLINE)", self.get_logger()):
                try:
                    ss = vamp.SimplifySettings()
                    simplified = vamp.frida_real.simplify(raw_path, env, ss, rng)
                    if simplified.solved and len(simplified.path) > 0:
                        path_nodes = [
                            np.array(list(simplified.path[i]), dtype=np.float64)
                            for i in range(len(simplified.path))
                        ]
                        self.get_logger().info(f"  Native simplify: {len(path_nodes)} waypoints")
                except Exception as e:
                    self.get_logger().warn(f"Native simplify failed: {e}")

            if path_nodes is None:
                self.get_logger().warn("Falling back to Python smoothing pipeline.")
                n_points = len(raw_path)
                step = max(1, n_points // 500)
                path_nodes = [np.array(list(raw_path[int(i)]), dtype=np.float64)
                              for i in range(0, n_points, step)]
                if (n_points - 1) % step != 0:
                    path_nodes.append(np.array(list(raw_path[-1]), dtype=np.float64))
                path_nodes = self.downsample_path(path_nodes)
                path_nodes = self.apply_smoothing_filter(path_nodes)

            with self.Timer("Final validation & Densify", self.get_logger()):
                final_dense_path, is_safe = self.validate_and_densify_path(path_nodes, env)
            if not is_safe:
                self.get_logger().warn("Path has collision, retrying without smoothing.")
                n_points = len(raw_path)
                step = max(1, n_points // 500)
                path_nodes = [np.array(list(raw_path[int(i)]), dtype=np.float64)
                              for i in range(0, n_points, step)]
                path_nodes = self.downsample_path(path_nodes)
                final_dense_path, is_safe = self.validate_and_densify_path(path_nodes, env)
            if not is_safe:
                self.get_logger().error("Path has internal collision. Planning failed.")
                response.success = False
                return response
            
            flat = []
            for wp in final_dense_path:
                for j in range(ARM_DOF):
                    flat.append(float(wp[j]))

            response.waypoints_flat = flat
            response.success = True

            total_ms = (time.perf_counter() - total_start) * 1000.0
            self.get_logger().info(
                f"SUCCESS | {len(final_dense_path)} waypoints | total={total_ms:.1f} ms"
            )
            self.get_logger().info("=" * 60)

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            self.get_logger().error(traceback.format_exc())
            response.success = False

        return response


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