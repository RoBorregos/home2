#!/usr/bin/env python3

"""
Follow Person Controller — PI + Base Velocity Feedforward

Rotates xArm6 joint1 to keep a tracked person centered in the camera image.
Compensates for base rotation via feedforward from /cmd_vel.
Uses xArm velocity control mode (mode 4) for direct joint velocity commands.

Usage:
    ros2 run task_manager follow_person_controller.py

    # Enable following (after tracker is running and target is set):
    ros2 service call /follow_person frida_interfaces/srv/FollowFace "{follow_face: true}"

    # Disable:
    ros2 service call /follow_person frida_interfaces/srv/FollowFace "{follow_face: false}"
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import JointState
from frida_interfaces.srv import FollowFace
from frida_pymoveit2.robots import xarm6
from xarm_msgs.srv import SetInt16, MoveVelocity

# Topic / service names
CENTROID_TOPIC = "/vision/tracker_centroid"
CMD_VEL_TOPIC = "/cmd_vel"
JOINT_STATES_TOPIC = "/joint_states"
FOLLOW_SERVICE = "/follow_person"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"
XARM_VELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"

TARGET_JOINT = "joint1"

# xArm modes
VELOCITY_MODE = 4   # Joint velocity control
MOVEIT_MODE = 1      # Servo / MoveIt mode


class FollowPersonController(Node):
    def __init__(self):
        super().__init__("follow_person_controller")
        cb_group = ReentrantCallbackGroup()

        # --- Declare parameters ---
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("ki", 0.1)
        self.declare_parameter("kff", 1.0)
        self.declare_parameter("dead_zone", 0.05)
        self.declare_parameter("max_velocity", 1.5)
        self.declare_parameter("joint1_min", -2.8)
        self.declare_parameter("joint1_max", -0.3)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("centroid_timeout", 1.5)
        self.declare_parameter("integral_clamp", 0.5)

        # --- State ---
        self.active = False
        self.centroid_x = 0.0
        self.centroid_time = 0.0
        self.base_omega_z = 0.0
        self.joint_positions = {}
        self.error_integral = 0.0

        # --- Subscribers ---
        self.create_subscription(
            Point, CENTROID_TOPIC, self._centroid_cb, 10,
            callback_group=cb_group,
        )
        self.create_subscription(
            Twist, CMD_VEL_TOPIC, self._cmd_vel_cb, 10,
            callback_group=cb_group,
        )
        self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._joint_states_cb, 10,
            callback_group=cb_group,
        )

        # --- xArm service clients ---
        self.mode_client = self.create_client(
            SetInt16, XARM_SETMODE_SERVICE, callback_group=cb_group,
        )
        self.state_client = self.create_client(
            SetInt16, XARM_SETSTATE_SERVICE, callback_group=cb_group,
        )
        self.velocity_client = self.create_client(
            MoveVelocity, XARM_VELOCITY_SERVICE, callback_group=cb_group,
        )

        # --- Follow service ---
        self.create_service(
            FollowFace, FOLLOW_SERVICE, self._follow_service_cb,
            callback_group=cb_group,
        )

        # --- Control timer ---
        rate = self.get_parameter("control_rate").value
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._control_loop, callback_group=cb_group)

        self.get_logger().info(
            f"Follow Person Controller ready (rate={rate} Hz, "
            f"kp={self.get_parameter('kp').value}, "
            f"ki={self.get_parameter('ki').value}, "
            f"kff={self.get_parameter('kff').value})"
        )

    # ── Callbacks ──────────────────────────────────────────────

    def _centroid_cb(self, msg: Point):
        self.centroid_x = msg.x
        self.centroid_time = time.time()
        self.get_logger().info(f"Centroid: {msg.x:.3f}", once=True)

    def _cmd_vel_cb(self, msg: Twist):
        self.base_omega_z = msg.angular.z

    def _joint_states_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in xarm6.joint_names():
                self.joint_positions[name] = pos
        if TARGET_JOINT in self.joint_positions:
            self.get_logger().info(
                f"Joint states received (joint1={self.joint_positions[TARGET_JOINT]:.3f})",
                once=True,
            )

    def _follow_service_cb(self, request, response):
        self.active = request.follow_face
        if self.active:
            self.error_integral = 0.0
            self._set_arm_mode(VELOCITY_MODE)
            self.get_logger().info("Following enabled (velocity mode)")
        else:
            self._send_joint_velocity(0.0)
            time.sleep(0.3)
            self.error_integral = 0.0
            self._set_arm_mode(MOVEIT_MODE)
            self.get_logger().info("Following disabled (back to MoveIt mode)")
        response.success = True
        return response

    def _set_arm_mode(self, mode: int):
        """Set xArm mode + state 0."""
        try:
            mode_req = SetInt16.Request()
            mode_req.data = mode
            self.mode_client.call_async(mode_req)
            time.sleep(0.5)

            state_req = SetInt16.Request()
            state_req.data = 0
            self.state_client.call_async(state_req)
            time.sleep(0.5)

            self.get_logger().info(f"Arm mode set to {mode}")
        except Exception as e:
            self.get_logger().error(f"Failed to set arm mode: {e}")

    # ── Control loop ───────────────────────────────────────────

    def _control_loop(self):
        if not self.active:
            return

        if TARGET_JOINT not in self.joint_positions:
            return

        current_j1 = self.joint_positions[TARGET_JOINT]

        # Read parameters
        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kff = self.get_parameter("kff").value
        dead_zone = self.get_parameter("dead_zone").value
        max_vel = self.get_parameter("max_velocity").value
        j1_min = self.get_parameter("joint1_min").value
        j1_max = self.get_parameter("joint1_max").value
        timeout = self.get_parameter("centroid_timeout").value
        integral_clamp = self.get_parameter("integral_clamp").value

        age = time.time() - self.centroid_time

        if self.centroid_time == 0.0 or age > timeout:
            self.error_integral = 0.0
            self._send_joint_velocity(0.0)
            return

        # Error: positive centroid_x = person right = positive error
        # Sign is flipped in _send_joint_velocity (matching temp_follow.py)
        error = self.centroid_x

        # Dead zone
        if abs(error) < dead_zone:
            error = 0.0

        # PI controller
        self.error_integral += error * self.dt
        self.error_integral = max(-integral_clamp, min(integral_clamp, self.error_integral))

        # Anti-windup at joint limits
        if current_j1 <= j1_min or current_j1 >= j1_max:
            self.error_integral = 0.0

        pid_output = kp * error + ki * self.error_integral

        # Feedforward: compensate base rotation
        feedforward = self.base_omega_z * kff

        # Combined velocity
        joint1_vel = pid_output + feedforward
        joint1_vel = max(-max_vel, min(max_vel, joint1_vel))

        # Soft stop near joint limits
        if current_j1 <= j1_min and joint1_vel < 0:
            joint1_vel = 0.0
        if current_j1 >= j1_max and joint1_vel > 0:
            joint1_vel = 0.0

        self.get_logger().info(
            f"j1={current_j1:.3f} cx={self.centroid_x:.3f} "
            f"pid={pid_output:.3f} ff={feedforward:.3f} vel={joint1_vel:.3f}",
            throttle_duration_sec=0.5,
        )
        self._send_joint_velocity(joint1_vel)

    # ── Velocity command ───────────────────────────────────────

    def _send_joint_velocity(self, velocity: float):
        """Send joint velocity command — only joint1, all others 0."""
        req = MoveVelocity.Request()
        req.is_sync = True
        req.speeds = [-velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        try:
            future = self.velocity_client.call_async(req)
            future.add_done_callback(self._velocity_done_cb)
        except Exception as e:
            self.get_logger().error(f"Velocity command failed: {e}")

    def _velocity_done_cb(self, future):
        try:
            result = future.result()
            if result and not result.ret:
                pass  # success
        except Exception as e:
            self.get_logger().error(f"Velocity service error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FollowPersonController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
