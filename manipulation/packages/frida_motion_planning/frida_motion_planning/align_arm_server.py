#!/usr/bin/env python3

"""Action server that points the xArm6 at a 3D centroid.

Given a `PointStamped` in any TF-reachable frame, the server:
  1. Moves the arm to `pre_pose` (a named pose from `XARM_CONFIGURATIONS`).
  2. Reads the current joints; those of j3, j4, j6 act as the LOCK reference.
  3. Solves (j1, j2, j5) so both the arm shaft and the gripper approach axis
     point at the centroid.
  4. Sends the resulting joints back to the `MoveJoints` action server.

The cinematics solver is inlined here (rather than calling MoveIt) because
the FK chain is short and `np.linalg`-fast; we run it many times per goal.
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from frida_constants.manipulation_constants import (
    ALIGN_ARM_TO_CENTROID_ACTION_SERVER,
    GET_JOINT_SERVICE,
    MOVE_JOINTS_ACTION_SERVER,
)
from frida_interfaces.action import AlignArmToCentroid, MoveJoints
from frida_interfaces.srv import GetJoints
from frida_motion_planning.utils.service_utils import (
    get_joint_positions,
    move_joint_positions,
)
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point


# --- xArm6 + FRIDA mount FK constants (from the URDF) -----------------------

_XARM_MOUNT_XYZ = (0.036105613, 0.0, 0.441)
_XARM_MOUNT_YAW = math.pi / 2
_J1_OFFSET = (0.0, 0.0, 0.267)
_J3_OFFSET = (0.0535, -0.2845, 0.0)
_J4_OFFSET = (0.0775, 0.3425, 0.0)
_J6_OFFSET = (0.076, 0.097, 0.0)
_GRASP_YAW = -math.pi / 4

# Soft limits stay inside xArm6 hardware (j2 ∈ [-118°, +120°],
# j5 ∈ [-97°, +178°]) so the solver never commands into a hard stop.
_J2_MIN = math.radians(-110)
_J2_MAX = math.radians(90)
_J5_MIN = math.radians(-90)
_J5_MAX = math.radians(115)

_SHOULDER_IN_BASE = np.array(
    [_XARM_MOUNT_XYZ[0], _XARM_MOUNT_XYZ[1], _XARM_MOUNT_XYZ[2] + _J1_OFFSET[2]]
)

_DEFAULT_PRE_POSE = "washing_machine_arrow_pose"
_DEFAULT_VELOCITY = 0.2


def _trans(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def _rot(axis: str, a: float) -> np.ndarray:
    T = np.eye(4)
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        T[1, 1], T[1, 2], T[2, 1], T[2, 2] = c, -s, s, c
    elif axis == "y":
        T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, s, -s, c
    else:
        T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    return T


def _fk_grasp_frame(
    j1: float, j2: float, j3: float, j4: float, j5: float, j6: float
) -> np.ndarray:
    """4x4 transform of `gripper_grasp_frame` expressed in `base_link`."""
    T = _trans(*_XARM_MOUNT_XYZ) @ _rot("z", _XARM_MOUNT_YAW)
    T = T @ _trans(*_J1_OFFSET) @ _rot("z", j1)
    T = T @ _rot("x", -math.pi / 2) @ _rot("z", j2)
    T = T @ _trans(*_J3_OFFSET) @ _rot("z", j3)
    T = T @ _trans(*_J4_OFFSET) @ _rot("x", -math.pi / 2) @ _rot("z", j4)
    T = T @ _rot("x", math.pi / 2) @ _rot("z", j5)
    T = T @ _trans(*_J6_OFFSET) @ _rot("x", -math.pi / 2) @ _rot("z", j6)
    T = T @ _rot("z", _GRASP_YAW)
    return T


def _best_alignment(
    direction_fn, target_unit: np.ndarray, lo: float, hi: float, samples: int
):
    """Pick the angle in [lo, hi] whose `direction_fn` has max cosine with
    `target_unit`. Returns (best_angle, best_cos, best_dir, extra_metric).
    """
    best_v: Optional[float] = None
    best_cos = -math.inf
    best_dir = None
    extra = 0.0
    for v in np.linspace(lo, hi, samples):
        direction, magnitude = direction_fn(float(v))
        if direction is None:
            continue
        cos = float(np.dot(direction, target_unit))
        if cos > best_cos:
            best_cos = cos
            best_v = float(v)
            best_dir = direction
            extra = magnitude
    return best_v, best_cos, best_dir, extra


def _solve_arrow_alignment(
    centroid_xyz: Tuple[float, float, float],
    j3_lock: float,
    j4_lock: float,
    j5_seed: float,
    j6_lock: float,
    sweep_points: int = 240,
    iterations: int = 3,
) -> Tuple[Optional[float], Optional[float], Optional[float], dict]:
    """Solve (j1, j2, j5) so the arm shaft AND the gripper approach axis both
    point at `centroid_xyz` (in `base_link`). j3, j4, j6 stay locked.

    Iterates j2 ↔ j5 because changing j5 shifts the wrist by ~12 cm, which
    desyncs the arm-shaft direction; re-solving j2 corrects it.
    """
    target_vec = np.array(centroid_xyz, dtype=float) - _SHOULDER_IN_BASE
    target_dist = float(np.linalg.norm(target_vec))
    if target_dist < 1e-6:
        return None, None, None, {"error": "centroid coincides with shoulder"}
    target_unit = target_vec / target_dist

    azimuth = math.atan2(target_unit[1], target_unit[0])
    j1 = -math.pi / 2 + azimuth

    def arm_dir(j2: float, j5: float):
        T = _fk_grasp_frame(j1, j2, j3_lock, j4_lock, j5, j6_lock)
        v = T[:3, 3] - _SHOULDER_IN_BASE
        d = float(np.linalg.norm(v))
        return (v / d, d) if d > 1e-6 else (None, 0.0)

    def approach_dir(j2: float, j5: float):
        T = _fk_grasp_frame(j1, j2, j3_lock, j4_lock, j5, j6_lock)
        n = float(np.linalg.norm(T[:3, 2]))
        return (T[:3, 2] / n, 0.0) if n > 1e-6 else (None, 0.0)

    j5_iter = float(j5_seed)
    j2_iter: Optional[float] = None
    arm_cos = approach_cos = -math.inf

    for _ in range(max(1, iterations)):
        j2_iter, arm_cos, _, _ = _best_alignment(
            lambda v: arm_dir(v, j5_iter),
            target_unit,
            _J2_MIN,
            _J2_MAX,
            sweep_points,
        )
        if j2_iter is None or arm_cos < 0.0:
            return None, None, None, {"arm_cos": arm_cos, "error": "no forward j2"}
        j5_iter, approach_cos, _, _ = _best_alignment(
            lambda v: approach_dir(j2_iter, v),
            target_unit,
            _J5_MIN,
            _J5_MAX,
            sweep_points,
        )
        if j5_iter is None or approach_cos < 0.0:
            return (
                None,
                None,
                None,
                {
                    "arm_cos": arm_cos,
                    "approach_cos": approach_cos,
                    "error": "no forward j5",
                },
            )

    info = {
        "target_dist": target_dist,
        "arm_cos": arm_cos,
        "approach_cos": approach_cos,
    }
    return float(j1), float(j2_iter), float(j5_iter), info


# --- ROS node ---------------------------------------------------------------


class AlignArmServer(Node):
    """Hosts the `AlignArmToCentroid` action.

    Re-uses the existing MoveJoints action and GetJoints service exposed by
    `motion_planning_server`, so this node only adds the math + orchestration
    and does NOT duplicate motion-planning machinery.
    """

    TARGET_FRAME = "base_link"

    def __init__(self) -> None:
        super().__init__("align_arm_server")
        self._cbg = ReentrantCallbackGroup()

        self._move_joints_client = ActionClient(
            self,
            MoveJoints,
            MOVE_JOINTS_ACTION_SERVER,
            callback_group=self._cbg,
        )
        self._get_joints_client = self.create_client(
            GetJoints,
            GET_JOINT_SERVICE,
            callback_group=self._cbg,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._action_server = ActionServer(
            self,
            AlignArmToCentroid,
            ALIGN_ARM_TO_CENTROID_ACTION_SERVER,
            self._execute,
            callback_group=self._cbg,
        )
        self.get_logger().info(
            f"AlignArmServer ready on {ALIGN_ARM_TO_CENTROID_ACTION_SERVER}"
        )

    def _execute(self, goal_handle):
        request = goal_handle.request
        result = AlignArmToCentroid.Result()
        result.success = False
        result.arm_cos = 0.0
        result.approach_cos = 0.0

        pre_pose = request.pre_pose or _DEFAULT_PRE_POSE
        velocity = request.velocity if request.velocity > 0.0 else _DEFAULT_VELOCITY

        self.get_logger().info(
            f"AlignArmToCentroid: pre_pose='{pre_pose}', velocity={velocity:.2f}"
        )

        # 1. Move to pre-pose so the joints we lock have known values.
        if not move_joint_positions(
            self._move_joints_client,
            named_position=pre_pose,
            velocity=0.3,
        ):
            self.get_logger().error(f"Failed to reach pre-pose '{pre_pose}'")
            goal_handle.abort()
            return result

        rclpy.spin_once(self, timeout_sec=0.5)

        # 2. Read locked joint values from the live robot state.
        current = get_joint_positions(self._get_joints_client)
        if not isinstance(current, dict) or "joint3" not in current:
            self.get_logger().error(f"GetJoints returned bad payload: {current}")
            goal_handle.abort()
            return result
        j3, j4, j5_seed, j6 = (
            current["joint3"],
            current["joint4"],
            current["joint5"],
            current["joint6"],
        )

        # 3. Transform centroid into base_link.
        point = request.point
        if point is None or point.header.frame_id == "":
            self.get_logger().error("Invalid centroid (empty frame_id)")
            goal_handle.abort()
            return result
        try:
            tf = self._tf_buffer.lookup_transform(
                self.TARGET_FRAME,
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            point_in = do_transform_point(point, tf)
        except Exception as e:
            self.get_logger().error(
                f"TF {point.header.frame_id} -> {self.TARGET_FRAME}: {e}"
            )
            goal_handle.abort()
            return result

        cx, cy, cz = (
            float(point_in.point.x),
            float(point_in.point.y),
            float(point_in.point.z),
        )
        self.get_logger().info(
            f"Centroid in {self.TARGET_FRAME}=({cx:.3f}, {cy:.3f}, {cz:.3f}); "
            f"locked (j3,j4,j5,j6)=({j3:.2f}, {j4:.2f}, {j5_seed:.2f}, {j6:.2f}) rad"
        )

        # 4. Solve.
        j1, j2, j5_solved, info = _solve_arrow_alignment(
            (cx, cy, cz),
            j3_lock=j3,
            j4_lock=j4,
            j5_seed=j5_seed,
            j6_lock=j6,
        )
        if j1 is None or j2 is None or j5_solved is None:
            self.get_logger().error(
                f"Solver failed: arm_cos={info.get('arm_cos'):.3f}, "
                f"approach_cos={info.get('approach_cos'):.3f}"
            )
            goal_handle.abort()
            return result

        if j1 > math.pi:
            j1 -= 2 * math.pi
        elif j1 < -math.pi:
            j1 += 2 * math.pi
        if not (-math.pi <= j1 <= math.pi):
            self.get_logger().error(f"j1={j1:.3f} rad out of [-π, π]")
            goal_handle.abort()
            return result
        if not (_J2_MIN <= j2 <= _J2_MAX):
            self.get_logger().error(
                f"j2={j2:.3f} rad out of [{_J2_MIN:.3f}, {_J2_MAX:.3f}]"
            )
            goal_handle.abort()
            return result
        if not (_J5_MIN <= j5_solved <= _J5_MAX):
            self.get_logger().error(
                f"j5={j5_solved:.3f} rad out of [{_J5_MIN:.3f}, {_J5_MAX:.3f}]"
            )
            goal_handle.abort()
            return result

        T = _fk_grasp_frame(j1, j2, j3, j4, j5_solved, j6)
        tip = T[:3, 3]
        approach = T[:3, 2]
        self.get_logger().info(
            f"Solved: j1={math.degrees(j1):+.1f}°, j2={math.degrees(j2):+.1f}°, "
            f"j5={math.degrees(j5_solved):+.1f}°; tip~({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}); "
            f"approach=({approach[0]:+.2f}, {approach[1]:+.2f}, {approach[2]:+.2f}); "
            f"arm_cos={info['arm_cos']:.3f}, approach_cos={info['approach_cos']:.3f}"
        )

        # 5. Execute final joint goal.
        target_joints = {
            "joints": {
                "joint1": j1,
                "joint2": j2,
                "joint3": j3,
                "joint4": j4,
                "joint5": j5_solved,
                "joint6": j6,
            },
            "degrees": False,
        }
        if not move_joint_positions(
            self._move_joints_client,
            joint_positions=target_joints,
            velocity=velocity,
        ):
            self.get_logger().error("Final MoveJoints failed")
            goal_handle.abort()
            return result

        result.success = True
        result.arm_cos = float(info["arm_cos"])
        result.approach_cos = float(info["approach_cos"])
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = AlignArmServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
