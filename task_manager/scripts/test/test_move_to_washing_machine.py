#!/usr/bin/env python3

"""
End-to-end test: get washing-machine drum centroid via moondream and reach the
centroid HEIGHT using a joint-space strategy with only joint1, joint2 and joint5.

Strategy (single shot, closed-form):
  * joint1: rotated so the arm pans toward the centroid azimuth.
  * joint2: solved via FK so the gripper_grasp_frame reaches the centroid z.
            (joints 3, 4, 6 stay locked at their pre-pose values).
  * joint5: solved so the gripper approach axis ends horizontal w.r.t. base.

FK is computed from the URDF parameters (xArm6 + FRIDA mount + grasp frame),
so the result doesn't depend on empirical gains.
"""

import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_geometry_msgs import do_transform_point  # noqa: F401

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

VELOCITY = 0.2
CAMERA_FLIP = False
PRE_POSE = "table_stare"
WRIST_FRAME = "gripper_grasp_frame"
SUBJECT = (
    "exact geometric center of the circular washing machine drum opening "
    "(the round hole in the front door where clothes go in); point at the "
    "middle of the circle, not the door, rim, glass, or     surrounding frame"
)

# Soft limits to keep the solver away from hard joint stops.
J2_MIN = math.radians(-110)
J2_MAX = math.radians(20)
J5_MIN = math.radians(-115)
J5_MAX = math.radians(115)

# ----- xArm6 + FRIDA URDF constants (m / rad) ------------------------------
# base_link -> xarm_base: trans(0.036, 0, 0.441), rpy=(0,0,π/2)
XARM_MOUNT_XYZ = (0.036105613, 0.0, 0.441)
XARM_MOUNT_YAW = math.pi / 2
# xarm_base -> link1 (joint1): trans(0,0,0.267)
J1_OFFSET = (0.0, 0.0, 0.267)
# link1 -> link2 (joint2): rpy=(-π/2,0,0)
# link2 -> link3 (joint3): trans(0.0535, -0.2845, 0)
J3_OFFSET = (0.0535, -0.2845, 0.0)
# link3 -> link4 (joint4): trans(0.0775, 0.3425, 0), rpy=(-π/2,0,0)
J4_OFFSET = (0.0775, 0.3425, 0.0)
# link4 -> link5 (joint5): rpy=(π/2,0,0)
# link5 -> link6 (joint6): trans(0.076, 0.097, 0), rpy=(-π/2,0,0)
J6_OFFSET = (0.076, 0.097, 0.0)
# link_eef -> gripper_grasp_frame: rpy=(0,0,-π/4)
GRASP_YAW = -math.pi / 4


def _T_trans(x, y, z):
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def _T_rot(axis, a):
    T = np.eye(4)
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        T[1, 1], T[1, 2], T[2, 1], T[2, 2] = c, -s, s, c
    elif axis == "y":
        T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, s, -s, c
    else:  # z
        T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    return T


def fk_grasp_frame(j1, j2, j3, j4, j5, j6) -> np.ndarray:
    """Forward kinematics: 4x4 transform of gripper_grasp_frame in base_link."""
    T = _T_trans(*XARM_MOUNT_XYZ) @ _T_rot("z", XARM_MOUNT_YAW)  # base_link -> xarm_base
    T = T @ _T_trans(*J1_OFFSET) @ _T_rot("z", j1)  # -> link1
    T = T @ _T_rot("x", -math.pi / 2) @ _T_rot("z", j2)  # -> link2
    T = T @ _T_trans(*J3_OFFSET) @ _T_rot("z", j3)  # -> link3
    T = T @ _T_trans(*J4_OFFSET) @ _T_rot("x", -math.pi / 2) @ _T_rot("z", j4)  # -> link4
    T = T @ _T_rot("x", math.pi / 2) @ _T_rot("z", j5)  # -> link5
    T = (
        T @ _T_trans(*J6_OFFSET) @ _T_rot("x", -math.pi / 2) @ _T_rot("z", j6)
    )  # -> link6 = link_eef
    T = T @ _T_rot("z", GRASP_YAW)  # -> gripper_grasp_frame
    return T


def _bisect(fn, lo, hi, target=0.0, tol=1e-4, max_iter=80):
    """Find root of fn(x) - target in [lo, hi]. Returns x or None."""
    flo = fn(lo) - target
    fhi = fn(hi) - target
    if flo * fhi > 0:
        return None
    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        fmid = fn(mid) - target
        if abs(fmid) < tol:
            return mid
        if flo * fmid < 0:
            hi = mid
            fhi = fmid
        else:
            lo = mid
            flo = fmid
    return 0.5 * (lo + hi)


class TestMoveToWashingMachine(Node):
    def __init__(self):
        super().__init__("test_move_to_washing_machine")
        self.vision = VisionTasks(self, task=Task.DEBUG)
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.vision.camera_upside_down(CAMERA_FLIP)

    def run(self) -> int:
        # --- 1. Pre-pose ---------------------------------------------------
        Logger.info(self, f"Pre-positioning to {PRE_POSE}...")
        if (
            self.manipulation.move_joint_positions(named_position=PRE_POSE, velocity=0.3)
            != Status.EXECUTION_SUCCESS
        ):
            return 3

        # --- 2. Read pre-pose joint values (lock reference) ----------------
        rclpy.spin_once(self, timeout_sec=0.5)
        current = self.manipulation.get_joint_positions()
        if not isinstance(current, dict):
            Logger.error(self, f"Bad current joints: {current}")
            return 5
        j3 = current["joint3"]
        j4 = current["joint4"]
        j6 = current["joint6"]

        # Sanity: FK of the pre-pose should match TF's reading of WRIST_FRAME z
        T_pre = fk_grasp_frame(
            current["joint1"],
            current["joint2"],
            j3,
            j4,
            current["joint5"],
            j6,
        )
        fk_z = T_pre[2, 3]
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link",
                WRIST_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            tf_z = tf.transform.translation.z
        except Exception as e:
            tf_z = float("nan")
            Logger.warn(self, f"TF lookup failed: {e}")
        Logger.info(
            self,
            f"Pre-pose FK z={fk_z:.3f}, TF z={tf_z:.3f} " f"(should agree within ~1 cm)",
        )

        # --- 3. Centroid from vision --------------------------------------
        Logger.info(self, "Requesting centroid from moondream...")
        point = self.vision.get_moondream_point_3d(SUBJECT)
        if point is None:
            Logger.error(self, "No centroid. Aborting.")
            return 1
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            point_base = do_transform_point(point, transform)
        except Exception as e:
            Logger.error(self, f"TF to base_link failed: {e}")
            return 4
        cx, cy, cz = point_base.point.x, point_base.point.y, point_base.point.z
        Logger.success(self, f"Centroid in base_link: ({cx:.3f}, {cy:.3f}, {cz:.3f})")

        # --- 4. joint1: pan toward azimuth --------------------------------
        # At j1 = -π/2 the (base_link -> link1) net z-rotation is 0
        # (XARM_MOUNT_YAW=+π/2 cancels), so the arm operates in +X of base_link.
        azimuth = math.atan2(cy, cx)
        j1 = -math.pi / 2 + azimuth
        Logger.info(
            self,
            f"Azimuth {math.degrees(azimuth):+.1f}° -> " f"joint1 {math.degrees(j1):+.1f}°",
        )

        # --- 5. joint2: bisect for target height --------------------------
        j5_fixed_for_z_solve = current["joint5"]

        def wrist_z_at(j2):
            T = fk_grasp_frame(j1, j2, j3, j4, j5_fixed_for_z_solve, j6)
            return T[2, 3]

        # Sweep to find the j2 range that brackets cz.
        sweep = np.linspace(J2_MIN, J2_MAX, 40)
        zs = [wrist_z_at(j2) for j2 in sweep]
        Logger.info(
            self,
            f"j2 sweep z range: [{min(zs):.3f}, {max(zs):.3f}] m (target {cz:.3f})",
        )
        bracket = None
        for i in range(len(sweep) - 1):
            if (zs[i] - cz) * (zs[i + 1] - cz) <= 0:
                bracket = (sweep[i], sweep[i + 1])
                break
        if bracket is None:
            Logger.error(
                self,
                f"Target height {cz:.3f} m unreachable with j3,j4,j6 locked. "
                f"Closest z = {min(zs, key=lambda v: abs(v - cz)):.3f}",
            )
            # Fall back to the j2 that gets closest, so we still extend forward.
            j2 = sweep[int(np.argmin([abs(z - cz) for z in zs]))]
        else:
            j2 = _bisect(wrist_z_at, bracket[0], bracket[1], target=cz)
        j2 = max(J2_MIN, min(J2_MAX, j2))
        Logger.info(self, f"joint2 -> {math.degrees(j2):+.1f}°")

        # --- 6. joint5: bisect for horizontal gripper approach axis -------
        # The gripper approach is the Z column of T's rotation. We want it
        # horizontal: T[2, 2] == 0.
        def gripper_z_z_at(j5):
            T = fk_grasp_frame(j1, j2, j3, j4, j5, j6)
            return T[2, 2]

        j5 = _bisect(gripper_z_z_at, J5_MIN, J5_MAX, target=0.0)
        if j5 is None:
            # Pick the j5 with smallest |T[2,2]|
            j5_sweep = np.linspace(J5_MIN, J5_MAX, 60)
            j5 = j5_sweep[int(np.argmin([abs(gripper_z_z_at(v)) for v in j5_sweep]))]
            Logger.warn(self, "No exact horizontal j5; using closest.")
        j5 = max(J5_MIN, min(J5_MAX, j5))
        Logger.info(self, f"joint5 -> {math.degrees(j5):+.1f}°")

        # --- 7. Build and send -------------------------------------------
        new_joints = {
            "joint1": j1,
            "joint2": j2,
            "joint3": j3,
            "joint4": j4,
            "joint5": j5,
            "joint6": j6,
        }
        T_pred = fk_grasp_frame(j1, j2, j3, j4, j5, j6)
        Logger.info(
            self,
            f"Predicted gripper pos: ({T_pred[0, 3]:.3f}, "
            f"{T_pred[1, 3]:.3f}, {T_pred[2, 3]:.3f}), "
            f"approach Z = ({T_pred[0, 2]:.2f}, {T_pred[1, 2]:.2f}, "
            f"{T_pred[2, 2]:.2f})",
        )
        Logger.info(
            self,
            "New joints (deg): {"
            + ", ".join(f"{k}: {math.degrees(v):.1f}" for k, v in new_joints.items())
            + "}",
        )

        status = self.manipulation.move_joint_positions(
            joint_positions=new_joints, velocity=VELOCITY
        )
        if status == Status.EXECUTION_SUCCESS:
            rclpy.spin_once(self, timeout_sec=0.5)
            try:
                tf = self._tf_buffer.lookup_transform(
                    "base_link",
                    WRIST_FRAME,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0),
                )
                Logger.success(
                    self,
                    f"Move SUCCESS. Wrist now at "
                    f"({tf.transform.translation.x:.3f}, "
                    f"{tf.transform.translation.y:.3f}, "
                    f"{tf.transform.translation.z:.3f}); "
                    f"target z was {cz:.3f}",
                )
            except Exception as e:
                Logger.warn(self, f"Final TF lookup failed: {e}")
            return 0
        Logger.error(self, f"Move failed with status={status}")
        return 2


def main(args=None):
    rclpy.init(args=args)
    node = TestMoveToWashingMachine()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        try:
            node.vision.camera_upside_down(False)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
