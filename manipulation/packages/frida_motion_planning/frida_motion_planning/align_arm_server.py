#!/usr/bin/env python3

"""Action server that aims the xArm6 at a 3D centroid.

Given a `PointStamped`, runs FK + a brute-force j2/j5 sweep so the arm shaft
and the gripper approach axis both point at the target. j3, j4, j6 are read
from the live robot after moving to `pre_pose` and stay fixed during the
solve. Final joints are commanded through the existing `MoveJoints` action.
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


# xArm6 + FRIDA mount URDF chain constants.
_XARM_MOUNT_XYZ = (0.036105613, 0.0, 0.441)
_XARM_MOUNT_YAW = math.pi / 2
_J1_OFFSET = (0.0, 0.0, 0.267)
_J3_OFFSET = (0.0535, -0.2845, 0.0)
_J4_OFFSET = (0.0775, 0.3425, 0.0)
_J6_OFFSET = (0.076, 0.097, 0.0)
_GRASP_YAW = -math.pi / 4

# Soft limits inside the xArm6 hardware range.
_J2_MIN, _J2_MAX = math.radians(-110), math.radians(90)
_J5_MIN, _J5_MAX = math.radians(-90), math.radians(115)

_SHOULDER_IN_BASE = np.array(
    [_XARM_MOUNT_XYZ[0], _XARM_MOUNT_XYZ[1], _XARM_MOUNT_XYZ[2] + _J1_OFFSET[2]]
)

_DEFAULT_PRE_POSE = "washing_machine_arrow_pose"
_DEFAULT_VELOCITY = 0.2
_TARGET_FRAME = "base_link"


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


def _fk_grasp_frame(j1, j2, j3, j4, j5, j6) -> np.ndarray:
    """4x4 transform of `gripper_grasp_frame` in `base_link`."""
    T = _trans(*_XARM_MOUNT_XYZ) @ _rot("z", _XARM_MOUNT_YAW)
    T = T @ _trans(*_J1_OFFSET) @ _rot("z", j1)
    T = T @ _rot("x", -math.pi / 2) @ _rot("z", j2)
    T = T @ _trans(*_J3_OFFSET) @ _rot("z", j3)
    T = T @ _trans(*_J4_OFFSET) @ _rot("x", -math.pi / 2) @ _rot("z", j4)
    T = T @ _rot("x", math.pi / 2) @ _rot("z", j5)
    T = T @ _trans(*_J6_OFFSET) @ _rot("x", -math.pi / 2) @ _rot("z", j6)
    return T @ _rot("z", _GRASP_YAW)


def _argmax_cos(get_dir, target_unit, lo, hi, samples):
    """Sweep [lo, hi] and return (angle, cos) maximising dot(get_dir(a), target_unit)."""
    best_a, best_cos = None, -math.inf
    for v in np.linspace(lo, hi, samples):
        d = get_dir(float(v))
        if d is None:
            continue
        c = float(np.dot(d, target_unit))
        if c > best_cos:
            best_a, best_cos = float(v), c
    return best_a, best_cos


def _solve_arrow_alignment(
    centroid_xyz: Tuple[float, float, float],
    j3_lock: float,
    j4_lock: float,
    j5_seed: float,
    j6_lock: float,
    sweep_points: int = 240,
    iterations: int = 3,
) -> Tuple[Optional[float], Optional[float], Optional[float], dict]:
    """Solve (j1, j2, j5) so the arm shaft and the gripper approach axis both
    point at `centroid_xyz`. Iterates j2↔j5 because the wrist offset (~12 cm)
    couples them.
    """
    target_vec = np.array(centroid_xyz, dtype=float) - _SHOULDER_IN_BASE
    target_dist = float(np.linalg.norm(target_vec))
    if target_dist < 1e-6:
        return None, None, None, {"error": "centroid at shoulder"}
    target_unit = target_vec / target_dist

    j1 = -math.pi / 2 + math.atan2(target_unit[1], target_unit[0])

    def arm_dir(j2, j5):
        T = _fk_grasp_frame(j1, j2, j3_lock, j4_lock, j5, j6_lock)
        v = T[:3, 3] - _SHOULDER_IN_BASE
        d = float(np.linalg.norm(v))
        return v / d if d > 1e-6 else None

    def approach_dir(j2, j5):
        T = _fk_grasp_frame(j1, j2, j3_lock, j4_lock, j5, j6_lock)
        n = float(np.linalg.norm(T[:3, 2]))
        return T[:3, 2] / n if n > 1e-6 else None

    j5, j2 = float(j5_seed), None
    arm_cos = approach_cos = -math.inf
    for _ in range(max(1, iterations)):
        j2, arm_cos = _argmax_cos(
            lambda v: arm_dir(v, j5), target_unit, _J2_MIN, _J2_MAX, sweep_points
        )
        if j2 is None or arm_cos < 0.0:
            return None, None, None, {"arm_cos": arm_cos, "error": "no forward j2"}
        j5, approach_cos = _argmax_cos(
            lambda v: approach_dir(j2, v), target_unit, _J5_MIN, _J5_MAX, sweep_points
        )
        if j5 is None or approach_cos < 0.0:
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

    return (
        float(j1),
        float(j2),
        float(j5),
        {
            "arm_cos": arm_cos,
            "approach_cos": approach_cos,
            "target_dist": target_dist,
        },
    )


class AlignArmServer(Node):
    """Hosts the `AlignArmToCentroid` action; orchestrates MoveJoints +
    GetJoints exposed by `motion_planning_server`.
    """

    def __init__(self) -> None:
        super().__init__("align_arm_server")
        cbg = ReentrantCallbackGroup()
        self._move_joints_client = ActionClient(
            self, MoveJoints, MOVE_JOINTS_ACTION_SERVER, callback_group=cbg
        )
        self._get_joints_client = self.create_client(
            GetJoints, GET_JOINT_SERVICE, callback_group=cbg
        )
        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer, self)
        self._action_server = ActionServer(
            self,
            AlignArmToCentroid,
            ALIGN_ARM_TO_CENTROID_ACTION_SERVER,
            self._execute,
            callback_group=cbg,
        )
        self.get_logger().info(
            f"AlignArmServer ready on {ALIGN_ARM_TO_CENTROID_ACTION_SERVER}"
        )

    def _abort(self, goal_handle, result, msg):
        self.get_logger().error(msg)
        goal_handle.abort()
        return result

    def _execute(self, goal_handle):
        req = goal_handle.request
        result = AlignArmToCentroid.Result()
        result.success = False

        pre_pose = req.pre_pose or _DEFAULT_PRE_POSE
        velocity = req.velocity if req.velocity > 0.0 else _DEFAULT_VELOCITY
        self.get_logger().info(
            f"AlignArmToCentroid: pre_pose='{pre_pose}', v={velocity:.2f}"
        )

        if not move_joint_positions(
            self._move_joints_client, named_position=pre_pose, velocity=0.3
        ):
            return self._abort(
                goal_handle, result, f"Failed to reach pre-pose '{pre_pose}'"
            )

        rclpy.spin_once(self, timeout_sec=0.5)
        current = get_joint_positions(self._get_joints_client)
        if not isinstance(current, dict) or "joint3" not in current:
            return self._abort(goal_handle, result, f"GetJoints bad payload: {current}")
        j3, j4, j5_seed, j6 = (
            current["joint3"],
            current["joint4"],
            current["joint5"],
            current["joint6"],
        )

        point = req.point
        if point is None or point.header.frame_id == "":
            return self._abort(goal_handle, result, "Invalid centroid (empty frame_id)")
        try:
            tf = self._tf_buffer.lookup_transform(
                _TARGET_FRAME,
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            point_in = do_transform_point(point, tf)
        except Exception as e:
            return self._abort(
                goal_handle,
                result,
                f"TF {point.header.frame_id} -> {_TARGET_FRAME}: {e}",
            )

        cx, cy, cz = (
            float(point_in.point.x),
            float(point_in.point.y),
            float(point_in.point.z),
        )
        self.get_logger().info(
            f"Centroid=({cx:.3f}, {cy:.3f}, {cz:.3f}); locked "
            f"(j3,j4,j5,j6)=({j3:.2f}, {j4:.2f}, {j5_seed:.2f}, {j6:.2f}) rad"
        )

        j1, j2, j5_solved, info = _solve_arrow_alignment(
            (cx, cy, cz), j3_lock=j3, j4_lock=j4, j5_seed=j5_seed, j6_lock=j6
        )
        if j1 is None:
            return self._abort(
                goal_handle,
                result,
                f"Solver failed: arm_cos={info.get('arm_cos', 0):.3f}, "
                f"approach_cos={info.get('approach_cos', 0):.3f}",
            )

        if j1 > math.pi:
            j1 -= 2 * math.pi
        elif j1 < -math.pi:
            j1 += 2 * math.pi
        if not (-math.pi <= j1 <= math.pi):
            return self._abort(goal_handle, result, f"j1={j1:.3f} out of [-π, π]")
        if not (_J2_MIN <= j2 <= _J2_MAX):
            return self._abort(goal_handle, result, f"j2={j2:.3f} out of soft limits")
        if not (_J5_MIN <= j5_solved <= _J5_MAX):
            return self._abort(
                goal_handle, result, f"j5={j5_solved:.3f} out of soft limits"
            )

        self.get_logger().info(
            f"Solved: j1={math.degrees(j1):+.1f}°, j2={math.degrees(j2):+.1f}°, "
            f"j5={math.degrees(j5_solved):+.1f}°; "
            f"arm_cos={info['arm_cos']:.3f}, approach_cos={info['approach_cos']:.3f}"
        )

        target = {
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
            self._move_joints_client, joint_positions=target, velocity=velocity
        ):
            return self._abort(goal_handle, result, "Final MoveJoints failed")

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
