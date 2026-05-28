#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import SetInt16, SetInt16ById, Call as XArmCall
from controller_manager_msgs.srv import SwitchController
from frida_interfaces.action import MoveJoints
from frida_constants.manipulation_constants import (
    XARM_ROBOT_STATES_TOPIC,
    XARM_CLEAN_ERROR_SERVICE,
    XARM_MOTION_ENABLE_SERVICE,
    XARM_SETMODE_SERVICE,
    XARM_SETSTATE_SERVICE,
    ESTOP_TOPIC,
    MOVE_JOINTS_ACTION_SERVER,
    MANIPULATION_ENSURE_ARM_READY_SERVICE,
)
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)


class ManipulationSafeguard(Node):
    def __init__(self):
        super().__init__("manipulation_safeguard")
        self.callback_group = ReentrantCallbackGroup()

        self._arm_state: RobotMsg | None = None
        self._in_estop = False
        self._recovering = False

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

        self.create_service(
            Trigger,
            MANIPULATION_ENSURE_ARM_READY_SERVICE,
            self._handle_ensure_arm_ready,
            callback_group=self.callback_group,
        )

        self.create_timer(
            2.0, self._try_estop_recovery, callback_group=self.callback_group
        )

        self.get_logger().info("Manipulation Safeguard node started")

    def _handle_ensure_arm_ready(self, request, response):
        self._ensure_arm_ready()
        ok = (
            self._arm_state is not None
            and self._arm_state.state in (1, 2)
            and self._arm_state.err == 0
        )
        response.success = ok
        response.message = "" if ok else "arm not ready after recovery attempt"
        return response

    def _on_arm_state(self, msg: RobotMsg):
        self._arm_state = msg
        if (msg.state == 4 or msg.err != 0) and not self._in_estop:
            self._in_estop = True
            self.get_logger().warn(
                f"E-stop ACTIVATED (state={msg.state}, err={msg.err}) — broadcasting abort"
            )
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
                and self._arm_state.state not in (3, 4)
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
        req.id = 8
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
        needs_reinit = s.err != 0 or s.mode != 1
        needs_reenable = s.state in (3, 4)

        if not (needs_reinit or needs_reenable):
            return

        if s.err != 0:
            self.get_logger().warn(f"ensure_arm_ready -> err={s.err}, cleaning error")
            self._call_svc(
                self._clean_error_client, XArmCall.Request(), 5.0, "clean_error"
            )

        if needs_reinit:
            self.get_logger().warn(
                f"ensure_arm_ready -> mode={s.mode} (expected 1), reinit mode 1"
            )
            self._reinit_mode1()

        if needs_reenable:
            self.get_logger().warn(
                f"ensure_arm_ready -> state={s.state} (paused/stopped), re-enabling motion"
            )
            self._motion_enable()
            req = SetInt16.Request()
            req.data = 0
            self._call_svc(self._set_state_client, req, 5.0, "set_state(0)")

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
                and self._arm_state.state in (1, 2)
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
