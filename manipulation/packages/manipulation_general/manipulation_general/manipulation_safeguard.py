#!/usr/bin/env python3

import math
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, Trigger
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import SetInt16, SetInt16ById, Call as XArmCall, MoveJoint
from controller_manager_msgs.srv import SwitchController
from frida_interfaces.action import MoveJoints
from frida_constants.manipulation_constants import (
    XARM_ROBOT_STATES_TOPIC,
    XARM_CLEAN_ERROR_SERVICE,
    XARM_MOTION_ENABLE_SERVICE,
    XARM_SETMODE_SERVICE,
    XARM_SETSTATE_SERVICE,
    XARM_SET_SERVO_ANGLE_SERVICE,
    XARM_POSITION_MODE,
    ESTOP_TOPIC,
    MOVE_JOINTS_ACTION_SERVER,
    MANIPULATION_ENSURE_ARM_READY_SERVICE,
    MOVEIT_MODE,
    XARM_STATE_READY,
    XARM_STATE_MOVING,
    XARM_STATE_PAUSED,
    XARM_STATE_STOPPED,
    XARM_ALL_JOINTS_ID,
)
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)
from frida_pymoveit2.robots.xarm6 import (
    JOINT_POSITION_LIMITS,
    joint_names as xarm6_joint_names,
)

# Per-joint limits from frida_pymoveit2/robots/xarm6.py — same source MoveIt uses.
_JOINT_NAMES = xarm6_joint_names()
_BOUNDS_TOLERANCE = 0.11  # slightly above start_state_max_bounds_error (0.1 rad)


class ManipulationSafeguard(Node):
    def __init__(self):
        super().__init__("manipulation_safeguard")
        self.callback_group = ReentrantCallbackGroup()

        self._arm_state: RobotMsg | None = None
        self._in_estop = False
        self._recovering = False
        self._pending_target_angles: list[float] | None = None

        # Global opt-in (default off via FRIDA_ENABLE_SAFEGUARD; ROS param overrides).
        # Disabled = no autonomous estop/recovery; ensure_arm_ready stays available.
        env_default = os.environ.get("FRIDA_ENABLE_SAFEGUARD", "false")
        self._enable_safeguard = self.declare_parameter(
            "enable_safeguard", env_default
        ).get_parameter_value().string_value.strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )

        # Always subscribe so _arm_state is populated for the on-demand ensure_arm_ready
        # service; the autonomous estop broadcast in the callback is gated by the flag.
        self.create_subscription(
            RobotMsg,
            XARM_ROBOT_STATES_TOPIC,
            self._on_arm_state,
            10,
            callback_group=self.callback_group,
        )

        self._estop_pub = self.create_publisher(Bool, ESTOP_TOPIC, 10)

        self._clean_error_client = self.create_client(
            XArmCall, XARM_CLEAN_ERROR_SERVICE, callback_group=self.callback_group
        )
        self._motion_enable_client = self.create_client(
            SetInt16ById, XARM_MOTION_ENABLE_SERVICE, callback_group=self.callback_group
        )
        self._set_mode_client = self.create_client(
            SetInt16, XARM_SETMODE_SERVICE, callback_group=self.callback_group
        )
        self._set_state_client = self.create_client(
            SetInt16, XARM_SETSTATE_SERVICE, callback_group=self.callback_group
        )
        self._clear_octomap_client = self.create_client(
            Empty, "/clear_octomap", callback_group=self.callback_group
        )
        self.switch_controller_client = self.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
            callback_group=self.callback_group,
        )
        self._move_joints_client = ActionClient(
            self,
            MoveJoints,
            MOVE_JOINTS_ACTION_SERVER,
            callback_group=self.callback_group,
        )
        self._set_servo_angle_client = self.create_client(
            MoveJoint, XARM_SET_SERVO_ANGLE_SERVICE, callback_group=self.callback_group
        )

        self.create_service(
            Trigger,
            MANIPULATION_ENSURE_ARM_READY_SERVICE,
            self._handle_ensure_arm_ready,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            JointState,
            "/manipulation/joint_goal_target",
            self._on_joint_goal_target,
            1,
            callback_group=self.callback_group,
        )

        # Autonomous recovery timer only when enabled; inert mode never auto-moves the arm.
        if self._enable_safeguard:
            self.create_timer(
                2.0, self._try_estop_recovery, callback_group=self.callback_group
            )

        mode = (
            "full autonomous safeguard"
            if self._enable_safeguard
            else "INERT (ensure_arm_ready only; no autonomous estop or recovery)"
        )
        self.get_logger().info(
            f"Manipulation Safeguard node started: enable_safeguard={self._enable_safeguard} [{mode}]"
        )

    def _handle_ensure_arm_ready(self, request, response):
        self._ensure_arm_ready()
        ok = (
            self._arm_state is not None
            and self._arm_state.state in (XARM_STATE_READY, XARM_STATE_MOVING)
            and self._arm_state.err == 0
            and not self._joints_out_of_bounds()
        )
        response.success = ok
        response.message = "" if ok else "arm not ready after recovery attempt"
        return response

    def _on_joint_goal_target(self, msg: JointState):
        """Store the incoming joint target so _ensure_arm_ready can use it when OOB."""
        target = {name: pos for name, pos in zip(msg.name, msg.position)}
        self._pending_target_angles = [
            target.get(name, self._arm_state.angle[i] if self._arm_state else 0.0)
            for i, name in enumerate(_JOINT_NAMES)
        ]

    def _on_arm_state(self, msg: RobotMsg):
        self._arm_state = msg
        if not self._enable_safeguard:
            return  # inert: keep arm state for ensure_arm_ready, but never auto-estop
        is_fault = msg.state == XARM_STATE_STOPPED or msg.err != 0
        if is_fault and not self._in_estop:
            self._in_estop = True
            reason = f"state={msg.state}, err={msg.err}"
            self.get_logger().warn(f"E-stop ACTIVATED ({reason}) — broadcasting abort")
            self._estop_pub.publish(Bool(data=True))

    def _try_estop_recovery(self):
        if not self._in_estop or self._recovering:
            return
        self._recovering = True
        try:
            self.get_logger().info("E-stop active — attempting arm recovery...")
            self._ensure_arm_ready()
            if (
                self._arm_state
                and self._arm_state.state not in (XARM_STATE_PAUSED, XARM_STATE_STOPPED)
                and self._arm_state.err == 0
            ):
                self._in_estop = False
                self.get_logger().warn(
                    "E-stop CLEARED — arm recovered, going to table_stare"
                )
                self._estop_pub.publish(Bool(data=False))
                send_joint_goal(
                    move_joints_action_client=self._move_joints_client,
                    named_position="table_stare",
                    velocity=0.3,
                )
                self._call_svc(
                    self._clear_octomap_client, Empty.Request(), 5.0, "clear_octomap"
                )
        finally:
            self._recovering = False

    def _joints_out_of_bounds(self) -> list[int]:
        """Return indices of joints whose reported angle violates JOINT_POSITION_LIMITS."""
        if not self._arm_state or not self._arm_state.angle:
            return []
        bad = []
        for i, (name, angle) in enumerate(zip(_JOINT_NAMES, self._arm_state.angle)):
            lower, upper = JOINT_POSITION_LIMITS[name]
            if angle < lower - _BOUNDS_TOLERANCE or angle > upper + _BOUNDS_TOLERANCE:
                bad.append(i)
        return bad

    def _find_delta(self, raw: float, lower: float, upper: float) -> float | None:
        """Return the multiple of 2π closest to zero that brings raw into [lower, upper].
        Returns None if no such multiple exists (joint range < 2π with no valid landing)."""
        two_pi = 2 * math.pi
        n_lo = math.ceil((lower - raw) / two_pi)
        n_hi = math.floor((upper - raw) / two_pi)
        if n_lo > n_hi:
            return None
        # Pick the integer n in [n_lo, n_hi] closest to 0 to minimise travel
        n = min(range(n_lo, n_hi + 1), key=abs)
        return n * two_pi

    def _normalize_joints(self, bad_indices: list[int]):
        """Move out-of-bounds joints back within limits via xarm mode 0.
        Leaves the arm in mode 0 — caller is responsible for reinit and recovery."""
        self.get_logger().warn("Normalizing joints via direct xarm control (mode 0)...")

        deltas = [0.0] * len(self._arm_state.angle)
        for i in bad_indices:
            name = _JOINT_NAMES[i]
            lower, upper = JOINT_POSITION_LIMITS[name]
            raw = self._arm_state.angle[i]
            delta = self._find_delta(raw, lower, upper)
            if delta is None:
                self.get_logger().error(
                    f"{name} OOB ({raw:.4f} rad) — no multiple of 2π brings it into "
                    f"[{lower:.4f}, {upper:.4f}]. Skipping normalization for this joint."
                )
                return
            deltas[i] = delta
            self.get_logger().warn(
                f"{name} OOB ({raw:.4f} rad) — moving by {delta:.4f} rad "
                f"to {raw + delta:.4f} rad (limits [{lower:.4f}, {upper:.4f}])"
            )

        req_mode0 = SetInt16.Request()
        req_mode0.data = XARM_POSITION_MODE
        if not self._call_svc(
            self._set_mode_client, req_mode0, 5.0, "set_mode(0) for normalization"
        ):
            self.get_logger().error("Normalization aborted: failed to set mode 0.")
            return
        req_state0 = SetInt16.Request()
        req_state0.data = 0
        if not self._call_svc(
            self._set_state_client, req_state0, 5.0, "set_state(0) for normalization"
        ):
            self.get_logger().error("Normalization aborted: failed to set state 0.")
            return

        req_move = MoveJoint.Request()
        req_move.angles = [float(d) for d in deltas]
        req_move.speed = 1.5
        req_move.acc = 0.5
        req_move.wait = True
        req_move.timeout = 15.0
        req_move.relative = True
        self._call_svc(
            self._set_servo_angle_client,
            req_move,
            17.0,
            "set_servo_angle normalization",
        )
        self.get_logger().warn("Joint normalization move complete.")

    def _normalize_joints_to_target(self, target_angles: list[float]):
        """Move all joints directly to target_angles via xarm mode 0 (absolute positioning).
        Avoids the intermediate normalization-to-zero step when joints are OOB."""
        self.get_logger().warn(
            "OOB recovery: moving directly to joint target via mode 0 (skipping intermediate 0-rad step)..."
        )
        req_mode0 = SetInt16.Request()
        req_mode0.data = XARM_POSITION_MODE
        if not self._call_svc(
            self._set_mode_client, req_mode0, 5.0, "set_mode(0) for OOB recovery"
        ):
            self.get_logger().error("OOB recovery aborted: failed to set mode 0.")
            return
        req_state0 = SetInt16.Request()
        req_state0.data = 0
        if not self._call_svc(
            self._set_state_client, req_state0, 5.0, "set_state(0) for OOB recovery"
        ):
            self.get_logger().error("OOB recovery aborted: failed to set state 0.")
            return

        req_move = MoveJoint.Request()
        req_move.angles = [float(a) for a in target_angles]
        req_move.speed = 1.5
        req_move.acc = 0.5
        req_move.wait = True
        req_move.timeout = 30.0
        req_move.relative = False
        self._call_svc(
            self._set_servo_angle_client,
            req_move,
            32.0,
            "set_servo_angle to target (OOB recovery)",
        )
        self.get_logger().warn("OOB recovery direct move complete.")

    def _call_svc(self, client, request, timeout, label):
        elapsed = 0.0
        while not client.service_is_ready() and elapsed < timeout:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            elapsed += 0.1
        if not client.service_is_ready():
            self.get_logger().warn(f"{label} -> service not ready after {timeout}s")
            return False
        future = client.call_async(request)
        elapsed = 0.0
        while not future.done() and elapsed < timeout:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            elapsed += 0.1
        if not future.done():
            self.get_logger().error(f"{label} -> timed out after {timeout}s")
            return False
        return True

    def _switch_controller(self, deactivate: list, activate: list, label: str) -> bool:
        req = SwitchController.Request()
        req.deactivate_controllers = deactivate
        req.activate_controllers = activate
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self.switch_controller_client.call_async(req)
        start = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start).nanoseconds > 5e9:
                self.get_logger().error(f"{label} -> timeout")
                return False
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        result = future.result()
        if result and result.ok:
            self.get_logger().info(f"{label} -> ok")
            return True
        self.get_logger().error(f"{label} -> failed")
        return False

    def _reactivate_controller(self):
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "_reactivate_controller -> switch_controller not available"
            )
            return False
        self._switch_controller(
            ["xarm6_traj_controller"], [], "deactivate xarm6_traj_controller"
        )
        return self._switch_controller(
            [], ["xarm6_traj_controller"], "activate xarm6_traj_controller"
        )

    def _motion_enable(self) -> bool:
        req = SetInt16ById.Request()
        req.id = XARM_ALL_JOINTS_ID
        req.data = 1
        return self._call_svc(self._motion_enable_client, req, 5.0, "motion_enable")

    def _reinit_mode1(self):
        self._motion_enable()
        req_en = SetInt16.Request()
        req_en.data = 0
        self._call_svc(self._set_mode_client, SetInt16.Request(), 5.0, "set_mode(0)")
        self._call_svc(self._set_state_client, req_en, 5.0, "set_state(0)")
        req1 = SetInt16.Request()
        req1.data = 1
        self._call_svc(self._set_mode_client, req1, 5.0, "set_mode(1)")
        self._call_svc(self._set_state_client, req_en, 5.0, "set_state(0) post-mode1")

    def _ensure_arm_ready(self):
        if self._arm_state is None:
            self.get_logger().warn(
                "ensure_arm_ready -> no robot_states received yet, skipping check"
            )
            return

        s = self._arm_state
        needs_reinit = s.err != 0 or s.mode != MOVEIT_MODE
        needs_reenable = s.state in (XARM_STATE_PAUSED, XARM_STATE_STOPPED)
        bad_joints = self._joints_out_of_bounds()
        needs_normalize = bool(bad_joints)

        if not (needs_reinit or needs_reenable or needs_normalize):
            return

        if s.err != 0:
            self.get_logger().warn(f"ensure_arm_ready -> err={s.err}, cleaning error")
            self._call_svc(
                self._clean_error_client, XArmCall.Request(), 5.0, "clean_error"
            )

        if needs_reenable:
            self.get_logger().warn(
                f"ensure_arm_ready -> state={s.state} (paused/stopped), re-enabling motion"
            )
            self._motion_enable()
            req = SetInt16.Request()
            req.data = 0
            self._call_svc(self._set_state_client, req, 5.0, "set_state(0)")

        if needs_normalize:
            if self._pending_target_angles is not None:
                self._normalize_joints_to_target(self._pending_target_angles)
                self._pending_target_angles = None
            else:
                self._normalize_joints(bad_joints)
            needs_reinit = True  # arm is now in mode 0; must reinit to mode 1

        if needs_reinit:
            self.get_logger().warn(
                f"ensure_arm_ready -> mode={s.mode} (expected 1), reinit mode 1"
            )
            self._reinit_mode1()

        self._reactivate_controller()
        self._wait_for_arm_ready(timeout_sec=5.0)
        # Clear stale octomap voxels: when the arm stops mid-motion the camera captures the arm as an obstacle
        self._call_svc(
            self._clear_octomap_client, Empty.Request(), 5.0, "clear_octomap"
        )

    def _wait_for_arm_ready(self, timeout_sec: float = 5.0):
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while self.get_clock().now() < deadline:
            if (
                self._arm_state is not None
                and self._arm_state.state in (XARM_STATE_READY, XARM_STATE_MOVING)
                and self._arm_state.err == 0
            ):
                return
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        s = self._arm_state.state if self._arm_state else "?"
        self.get_logger().warn(
            f"ensure_arm_ready -> arm did not reach ready state within {timeout_sec}s (state={s})"
        )


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(3)
    node = ManipulationSafeguard()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
