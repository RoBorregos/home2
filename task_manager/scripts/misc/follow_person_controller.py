#!/usr/bin/env python3

"""
Follow Person Controller — PID + Base Velocity Feedforward

Rotates xArm6 joint1 to keep a tracked person centered in the camera image.
The derivative term is computed at centroid rate (in the subscriber, not the
control loop) and low-pass filtered — it anticipates a person walking across
the frame instead of lagging them by error/kp.
Compensates for base rotation via feedforward from /cmd_vel.
Uses xArm velocity control mode (mode 4) for direct joint velocity commands.
Near the joint1 limits the commanded velocity tapers linearly to zero
(soft-limit) instead of cutting hard — that keeps the wide range usable
without slamming the joint into its limit (the old xArm-fault mode).
On centroid timeout the arm slowly recenters to joint1_neutral so the camera
faces where the base (per person_goal_smoother's lost-person goal) is heading.

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
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from frida_interfaces.srv import FollowFace
from frida_pymoveit2.robots import xarm6
from xarm_msgs.srv import SetInt16, MoveVelocity
from frida_constants.vision_constants import CENTROID_TOPIC

# Topic / service names
CMD_VEL_TOPIC = "/cmd_vel"
JOINT_STATES_TOPIC = "/joint_states"
FOLLOW_SERVICE = "/follow_person"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"
XARM_VELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"

TARGET_JOINT = "joint1"

# xArm modes
VELOCITY_MODE = 4  # Joint velocity control
MOVEIT_MODE = 1  # Servo / MoveIt mode


class FollowPersonController(Node):
    def __init__(self):
        super().__init__("follow_person_controller")
        cb_group = ReentrantCallbackGroup()

        # --- Declare parameters ---
        # Gains act on the normalized centroid error in [-1, 1]. Retuned for
        # keeping up with a walking person (validate with follow_calibration.py):
        # kp up 1.0->1.8, new kd (lead/damping), deadzone down, max_vel up.
        self.declare_parameter("kp", 1.8)
        self.declare_parameter("ki", 0.1)
        self.declare_parameter("kd", 0.12)
        self.declare_parameter("kff", 1.0)
        self.declare_parameter("dead_zone", 0.03)
        self.declare_parameter("max_velocity", 1.2)
        # Wider pan range than the old -2.8..-0.5; safe because velocity now
        # TAPERS over soft_limit_margin before a limit instead of cutting hard
        # (it was the hard slam into the limit that faulted the xArm, not the
        # range itself). Neutral (forward) is joint1_neutral = -1.5707.
        self.declare_parameter("joint1_min", -3.05)
        self.declare_parameter("joint1_max", -0.2)
        self.declare_parameter("soft_limit_margin", 0.35)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("centroid_timeout", 1.5)
        self.declare_parameter("integral_clamp", 0.3)
        # On centroid timeout, slowly pan back to neutral so the camera faces
        # forward (where the lost-person nav goal is taking the base).
        self.declare_parameter("recenter_enabled", True)
        self.declare_parameter("recenter_velocity", 0.3)
        # Reactive "unload-the-arm" base yaw: when joint1 has panned off neutral
        # (person to the side), rotate the BASE so joint1 returns toward neutral
        # -> effectively unlimited pan and the person stays in the camera FOV.
        # NOTE: verify the SIGN of base_yaw_kp on the robot (flip if the base
        # turns the wrong way); joint1_neutral = forward-pointing joint1 value.
        # OFF by default: the reactive base-yaw overshoots/oscillates with the
        # current arm+base coupling — to be revisited (likely alongside re-acquisition).
        # Tuned-down gains kept for when it's re-enabled.
        self.declare_parameter("base_yaw_enabled", False)
        self.declare_parameter("base_yaw_kp", 0.5)
        self.declare_parameter("joint1_neutral", -1.5707)
        self.declare_parameter("base_yaw_max", 0.25)

        # --- State ---
        self.active = False
        self.centroid_x = 0.0
        self.centroid_time = 0.0
        self.base_omega_z = 0.0
        self.joint_positions = {}
        self.error_integral = 0.0
        # Centroid-rate derivative (filled by _centroid_cb, low-pass filtered).
        # Computing it in the 20 Hz control loop would alternate spike/zero
        # because the centroid arrives at its own rate.
        self.error_deriv = 0.0

        # --- Subscribers ---
        self.create_subscription(
            Point,
            CENTROID_TOPIC,
            self._centroid_cb,
            10,
            callback_group=cb_group,
        )
        self.create_subscription(
            Twist,
            CMD_VEL_TOPIC,
            self._cmd_vel_cb,
            10,
            callback_group=cb_group,
        )
        self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._joint_states_cb,
            10,
            callback_group=cb_group,
        )

        # --- Base-yaw publisher (reactive unload-the-arm) ---
        self.base_yaw_pub = self.create_publisher(Float64, "/follow/base_yaw", 10)

        # --- xArm service clients ---
        self.mode_client = self.create_client(
            SetInt16,
            XARM_SETMODE_SERVICE,
            callback_group=cb_group,
        )
        self.state_client = self.create_client(
            SetInt16,
            XARM_SETSTATE_SERVICE,
            callback_group=cb_group,
        )
        self.velocity_client = self.create_client(
            MoveVelocity,
            XARM_VELOCITY_SERVICE,
            callback_group=cb_group,
        )

        # --- Follow service ---
        self.create_service(
            FollowFace,
            FOLLOW_SERVICE,
            self._follow_service_cb,
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
            f"kd={self.get_parameter('kd').value}, "
            f"kff={self.get_parameter('kff').value}, "
            f"j1=[{self.get_parameter('joint1_min').value}, "
            f"{self.get_parameter('joint1_max').value}])"
        )

    # ── Callbacks ──────────────────────────────────────────────

    def _centroid_cb(self, msg: Point):
        now = time.time()
        if self.centroid_time > 0.0:
            dt = now - self.centroid_time
            if 0.005 < dt < 0.5:
                d = (msg.x - self.centroid_x) / dt
                # LPF (~1/3 weight on the new sample) tames per-frame bbox jitter
                self.error_deriv = 0.35 * d + 0.65 * self.error_deriv
            elif dt >= 0.5:
                self.error_deriv = 0.0  # stale gap — a finite diff would spike
        self.centroid_x = msg.x
        self.centroid_time = now
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
        if request.follow_face:
            self.error_integral = 0.0
            self.error_deriv = 0.0
            self.centroid_time = 0.0  # don't act on a centroid from a past run
            self._set_arm_mode(VELOCITY_MODE)
            # Activate only AFTER the mode switch: with the multithreaded
            # executor the control loop keeps ticking during the sleeps above,
            # and velocity commands before mode 4 error out on the xArm.
            self.active = True
            self.get_logger().info("Following enabled (velocity mode)")
        else:
            self.active = False
            self._send_joint_velocity(0.0)
            time.sleep(0.3)
            self.error_integral = 0.0
            self.error_deriv = 0.0
            self._set_arm_mode(MOVEIT_MODE)
            self.get_logger().info("Following disabled (back to MoveIt mode)")
        response.success = True
        return response

    def _set_arm_mode(self, mode: int):
        """Set xArm mode + state 0. Must set mode first, then state."""
        try:
            # Step 1: clear errors by setting state 0
            state_req = SetInt16.Request()
            state_req.data = 0
            self.state_client.call_async(state_req)
            time.sleep(0.5)

            # Step 2: set desired mode
            mode_req = SetInt16.Request()
            mode_req.data = mode
            self.mode_client.call_async(mode_req)
            time.sleep(0.5)

            # Step 3: set state 0 again to activate
            self.state_client.call_async(state_req)
            time.sleep(0.5)

            self.get_logger().info(f"Arm mode set to {mode}")
        except Exception as e:
            self.get_logger().error(f"Failed to set arm mode: {e}")

    # ── Control loop ───────────────────────────────────────────

    def _control_loop(self):
        if not self.active:
            self._publish_base_yaw(0.0)
            return

        if TARGET_JOINT not in self.joint_positions:
            self._publish_base_yaw(0.0)
            return

        current_j1 = self.joint_positions[TARGET_JOINT]

        # Read parameters
        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value
        kff = self.get_parameter("kff").value
        dead_zone = self.get_parameter("dead_zone").value
        max_vel = self.get_parameter("max_velocity").value
        timeout = self.get_parameter("centroid_timeout").value
        integral_clamp = self.get_parameter("integral_clamp").value

        age = time.time() - self.centroid_time

        if self.centroid_time == 0.0 or age > timeout:
            self.error_integral = 0.0
            self.error_deriv = 0.0
            self._publish_base_yaw(0.0)
            self._recenter_or_stop(current_j1)
            return

        # Error: positive centroid_x = person right = positive error
        # Sign is flipped in _send_joint_velocity (matching temp_follow.py)
        error = self.centroid_x

        # Dead zone (P/I only — the derivative keeps damping inside it)
        if abs(error) < dead_zone:
            error = 0.0

        # PID controller (derivative computed at centroid rate in _centroid_cb)
        self.error_integral += error * self.dt
        self.error_integral = max(-integral_clamp, min(integral_clamp, self.error_integral))

        pid_output = kp * error + ki * self.error_integral + kd * self.error_deriv

        # Feedforward: compensate base rotation
        feedforward = self.base_omega_z * kff

        # Combined velocity
        joint1_vel = pid_output + feedforward
        joint1_vel = max(-max_vel, min(max_vel, joint1_vel))

        # Soft limits: taper to zero across soft_limit_margin instead of a hard
        # cutoff, so the wide joint1 range never ends in a full-speed slam
        # (that slam was the old xArm fault mode).
        joint1_vel, limited = self._apply_soft_limit(current_j1, joint1_vel)
        if limited:
            self.error_integral = 0.0

        self.get_logger().info(
            f"j1={current_j1:.3f} cx={self.centroid_x:.3f} "
            f"pid={pid_output:.3f} ff={feedforward:.3f} vel={joint1_vel:.3f}",
            throttle_duration_sec=0.5,
        )
        self._send_joint_velocity(joint1_vel)

        # Reactive base yaw: rotate the base to bring joint1 back toward neutral
        # (unload the arm) so the person stays centred with unlimited pan range.
        if self.get_parameter("base_yaw_enabled").value:
            neutral = self.get_parameter("joint1_neutral").value
            byaw_kp = self.get_parameter("base_yaw_kp").value
            byaw_max = self.get_parameter("base_yaw_max").value
            base_yaw = byaw_kp * (current_j1 - neutral)
            base_yaw = max(-byaw_max, min(byaw_max, base_yaw))
        else:
            base_yaw = 0.0
        self._publish_base_yaw(base_yaw)

    def _apply_soft_limit(self, current_j1: float, cmd_vel: float):
        """Taper the commanded velocity to zero across soft_limit_margin before a
        joint1 limit. Returns (scaled_vel, was_limited). Sign note: the actual
        joint velocity is -cmd_vel (negated in _send_joint_velocity), so
        cmd_vel > 0 moves joint1 NEGATIVE (toward joint1_min)."""
        if cmd_vel == 0.0:
            return 0.0, False
        j1_min = self.get_parameter("joint1_min").value
        j1_max = self.get_parameter("joint1_max").value
        margin = self.get_parameter("soft_limit_margin").value
        if cmd_vel > 0:  # actual motion toward j1_min
            dist = current_j1 - j1_min
        else:  # actual motion toward j1_max
            dist = j1_max - current_j1
        if dist >= margin:
            return cmd_vel, False
        scale = max(0.0, dist / margin)
        return cmd_vel * scale, True

    def _recenter_or_stop(self, current_j1: float):
        """No centroid: either hold still (legacy) or pan slowly back to
        joint1_neutral so the camera faces where the base is heading (the
        smoother drives to the person's last-known position on loss)."""
        if not self.get_parameter("recenter_enabled").value:
            self._send_joint_velocity(0.0)
            self.get_logger().warn(
                "No centroid data — sending zero velocity", throttle_duration_sec=2.0
            )
            return
        neutral = self.get_parameter("joint1_neutral").value
        recenter_vel = self.get_parameter("recenter_velocity").value
        err = neutral - current_j1  # desired actual joint displacement
        if abs(err) < 0.05:
            self._send_joint_velocity(0.0)
            return
        actual_vel = max(-recenter_vel, min(recenter_vel, err))
        # command is negated by _send_joint_velocity => pass -actual_vel
        self._send_joint_velocity(-actual_vel)
        self.get_logger().warn(
            f"No centroid data — recentering joint1 ({current_j1:.2f} -> {neutral:.2f})",
            throttle_duration_sec=2.0,
        )

    def _publish_base_yaw(self, value: float):
        msg = Float64()
        msg.data = float(value)
        self.base_yaw_pub.publish(msg)

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
            if result is None:
                self.get_logger().error("Velocity service returned None")
            elif result.ret != 0:
                self.get_logger().warn(
                    f"Velocity service ret={result.ret}", throttle_duration_sec=2.0
                )
        except Exception as e:
            self.get_logger().error(f"Velocity service error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FollowPersonController()
    # Multithreaded: the follow service's mode-switch sleeps (~1.5 s) must not
    # stall the 20 Hz control loop or the centroid/cmd_vel subscribers.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
