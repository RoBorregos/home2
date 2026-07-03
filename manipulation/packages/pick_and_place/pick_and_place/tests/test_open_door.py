#!/usr/bin/env python3
"""Open-door end-to-end test.

1. Pick the door handle through the manipulation action server (the handle
   branch levers it down, pulls the door ajar and retreats).
2. Put the xArm in manual mode (gravity-compensated free drive) so the arm
   complies if the door brushes it.
3. Drive the omnibase along a circular arc to swing the door fully open.
4. Restore the arm to MoveIt mode.

Requires the full pick_and_place + perception stack running (see
pick_and_place.launch.py) and the omnidriver listening on /cmd_vel.

    ros2 run pick_and_place test_open_door.py
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from xarm_msgs.srv import SetInt16

from frida_constants.manipulation_constants import (
    MANIPULATION_ACTION_SERVER,
    MOVEIT_MODE,
    XARM_MANUAL_MODE,
    XARM_POSITION_MODE,
    XARM_SETMODE_SERVICE,
    XARM_SETSTATE_SERVICE,
)
from frida_constants.navigation_constants import CMD_VEL_TOPIC
from frida_interfaces.action import ManipulationAction
from frida_interfaces.msg import ManipulationTask

HANDLE_OBJECT_NAME = "door_handle"

# Generous cap: the handle flow includes two mode-5 switches + slow MoveIt moves
PICK_RESULT_TIMEOUT = 180.0  # s

CMD_VEL_RATE = 20.0  # Hz


class OpenDoorTest(Node):
    def __init__(self):
        super().__init__("open_door_test")

        # Arc geometry/speed, tunable per door without editing code.
        # radius ~ distance base center -> door hinge; positive angle swings
        # counter-clockwise (hinge on the robot's left), negative clockwise.
        self.declare_parameter("arc_radius", 0.6)  # m
        self.declare_parameter("arc_angle_deg", 90.0)  # signed
        self.declare_parameter("arc_speed", 0.1)  # m/s along the arc

        self._manipulation_client = ActionClient(
            self, ManipulationAction, MANIPULATION_ACTION_SERVER
        )
        self._set_mode_client = self.create_client(SetInt16, XARM_SETMODE_SERVICE)
        self._set_state_client = self.create_client(SetInt16, XARM_SETSTATE_SERVICE)
        self._cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)

    # ------------------------------------------------------------------
    # Step 1: pick the handle (arm levers it down and pulls the door ajar)
    # ------------------------------------------------------------------

    def pick_handle(self) -> bool:
        if not self._manipulation_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Manipulation action server not available")
            return False

        goal_msg = ManipulationAction.Goal()
        goal_msg.task_type = ManipulationTask.PICK
        goal_msg.pick_params.object_name = HANDLE_OBJECT_NAME

        self.get_logger().info(f"Sending pick request for '{HANDLE_OBJECT_NAME}'...")
        goal_future = self._manipulation_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Pick goal rejected / not accepted")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=PICK_RESULT_TIMEOUT
        )
        if result_future.result() is None:
            self.get_logger().error("Pick result timed out")
            return False

        success = bool(result_future.result().result.success)
        self.get_logger().info(f"Handle pick {'succeeded' if success else 'failed'}")
        return success

    # ------------------------------------------------------------------
    # Step 2/4: xArm mode switching (same set_mode + set_state(0) dance as
    # pick_server._set_xarm_mode)
    # ------------------------------------------------------------------

    def _call_set_int16(self, client, value: int) -> bool:
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"{client.srv_name} unavailable")
            return False
        request = SetInt16.Request()
        request.data = value
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result() is not None

    def set_xarm_mode(self, mode: int) -> bool:
        if not self._call_set_int16(self._set_mode_client, mode):
            return False
        time.sleep(0.3)
        if not self._call_set_int16(self._set_state_client, 0):
            return False
        self.get_logger().info(f"xArm mode -> {mode}, state 0")
        time.sleep(0.5)
        return True

    # ------------------------------------------------------------------
    # Step 3: swing the door open with the base (constant-curvature arc)
    # ------------------------------------------------------------------

    def _publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.x = float(vx)
        t.twist.linear.y = float(vy)
        t.twist.angular.z = float(wz)
        self._cmd_pub.publish(t)

    def drive_door_arc(self) -> bool:
        radius = self.get_parameter("arc_radius").value
        angle_deg = self.get_parameter("arc_angle_deg").value
        speed = self.get_parameter("arc_speed").value
        if radius <= 0.0 or speed <= 0.0 or angle_deg == 0.0:
            self.get_logger().error(
                f"Bad arc parameters: radius={radius} angle={angle_deg} speed={speed}"
            )
            return False

        # Constant curvature: vx forward, wz = vx / radius (sign = swing side).
        wz = np.sign(angle_deg) * speed / radius
        duration = abs(np.radians(angle_deg)) * radius / speed
        self.get_logger().info(
            f"Driving door-open arc: radius={radius:.2f}m angle={angle_deg:.0f}deg "
            f"speed={speed:.2f}m/s (~{duration:.1f}s)"
        )

        dt = 1.0 / CMD_VEL_RATE
        t_end = time.time() + duration
        try:
            while time.time() < t_end:
                self._publish_cmd(vx=speed, wz=wz)
                time.sleep(dt)
        finally:
            self._publish_cmd(0.0, 0.0, 0.0)
        self.get_logger().info("Arc complete, base stopped")
        return True

    # ------------------------------------------------------------------

    def run(self) -> bool:
        if not self.pick_handle():
            return False

        self.get_logger().info("Putting arm in manual mode for the door swing...")
        if not self.set_xarm_mode(XARM_MANUAL_MODE):
            return False

        ok = self.drive_door_arc()

        # Restore MoveIt control (mode 0 -> 1, same dance as _restore_mode1)
        self.get_logger().info("Restoring arm to MoveIt mode...")
        self.set_xarm_mode(XARM_POSITION_MODE)
        time.sleep(0.5)
        self.set_xarm_mode(MOVEIT_MODE)
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = OpenDoorTest()
    try:
        success = node.run()
        node.get_logger().info(
            "Open door test PASSED" if success else "Open door test FAILED"
        )
    except KeyboardInterrupt:
        node._publish_cmd(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
