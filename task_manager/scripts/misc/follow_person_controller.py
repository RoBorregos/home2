#!/usr/bin/env python3

"""
Follow Person Controller — PI + Base Velocity Feedforward

Rotates xArm6 joint1 to keep a tracked person centered in the camera image.
Compensates for base rotation via feedforward from /cmd_vel.

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
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from frida_interfaces.srv import FollowFace
from frida_pymoveit2.robots import xarm6
from xarm_msgs.srv import SetInt16


# Topic names
CENTROID_TOPIC = "/vision/tracker_centroid"
CMD_VEL_TOPIC = "/cmd_vel"
JOINT_STATES_TOPIC = "/joint_states"
TRAJ_CONTROLLER_TOPIC = "/xarm6_traj_controller/joint_trajectory"
FOLLOW_SERVICE = "/follow_person"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"

# Joint name to control
TARGET_JOINT = "joint1"

# xArm modes
SERVO_MODE = 1  # Servo motion mode (for trajectory controller / MoveIt)


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
        self.declare_parameter("home_position", -1.57)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("centroid_timeout", 1.5)
        self.declare_parameter("integral_clamp", 0.5)
        self.declare_parameter("home_return_speed", 0.5)

        # --- State ---
        self.active = False
        self.centroid_x = 0.0
        self.centroid_time = 0.0
        self.base_omega_z = 0.0
        self.joint_positions = {}  # name -> position
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

        qos_joint = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._joint_states_cb, qos_joint,
            callback_group=cb_group,
        )

        # --- Publisher (RELIABLE to match trajectory controller) ---
        self.traj_pub = self.create_publisher(
            JointTrajectory, TRAJ_CONTROLLER_TOPIC, 10,
        )

        # --- xArm mode/state services ---
        self.mode_client = self.create_client(
            SetInt16, XARM_SETMODE_SERVICE, callback_group=cb_group,
        )
        self.state_client = self.create_client(
            SetInt16, XARM_SETSTATE_SERVICE, callback_group=cb_group,
        )

        # --- Service ---
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

    def _cmd_vel_cb(self, msg: Twist):
        self.base_omega_z = msg.angular.z

    def _joint_states_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in xarm6.joint_names():
                self.joint_positions[name] = pos

    def _follow_service_cb(self, request, response):
        self.active = request.follow_face
        if self.active:
            self.error_integral = 0.0
            self._set_arm_servo_mode()
            self.get_logger().info("Following enabled")
        else:
            self.error_integral = 0.0
            self._set_arm_servo_mode()
            self.get_logger().info("Following disabled")
        response.success = True
        return response

    def _set_arm_servo_mode(self):
        """Set xArm to servo mode (mode 1) + state 0 so trajectory controller works."""
        try:
            mode_req = SetInt16.Request()
            mode_req.data = SERVO_MODE
            self.mode_client.call_async(mode_req)

            state_req = SetInt16.Request()
            state_req.data = 0
            self.state_client.call_async(state_req)
            self.get_logger().info("Arm set to servo mode")
        except Exception as e:
            self.get_logger().error(f"Failed to set arm mode: {e}")

    # ── Control loop ───────────────────────────────────────────

    def _control_loop(self):
        if not self.active:
            return

        if TARGET_JOINT not in self.joint_positions:
            return

        current_j1 = self.joint_positions[TARGET_JOINT]

        # Read parameters (allows live tuning via ros2 param set)
        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kff = self.get_parameter("kff").value
        dead_zone = self.get_parameter("dead_zone").value
        max_vel = self.get_parameter("max_velocity").value
        j1_min = self.get_parameter("joint1_min").value
        j1_max = self.get_parameter("joint1_max").value
        home_pos = self.get_parameter("home_position").value
        timeout = self.get_parameter("centroid_timeout").value
        integral_clamp = self.get_parameter("integral_clamp").value
        home_speed = self.get_parameter("home_return_speed").value

        age = time.time() - self.centroid_time

        if self.centroid_time == 0.0 or age > timeout:
            # No recent tracker data — return to home
            self.error_integral = 0.0
            diff = home_pos - current_j1
            vel = max(-home_speed, min(home_speed, diff / 0.5))
            new_pos = current_j1 + vel * self.dt
            new_pos = max(j1_min, min(j1_max, new_pos))
            self._publish_joint1(new_pos, vel)
            return

        # Compute error (negative because positive centroid_x = person right
        # = need joint1 to decrease to pan camera right)
        error = -self.centroid_x

        # Dead zone
        if abs(error) < dead_zone:
            error = 0.0

        # PI
        self.error_integral += error * self.dt
        self.error_integral = max(-integral_clamp, min(integral_clamp, self.error_integral))

        # Anti-windup: reset integral if at joint limits
        if current_j1 <= j1_min or current_j1 >= j1_max:
            self.error_integral = 0.0

        pid_output = kp * error + ki * self.error_integral

        # Feedforward: compensate base rotation
        # When base rotates at +omega_z (CCW), joint1 must rotate at -omega_z
        # (CW relative to base) to keep camera pointing the same world direction
        feedforward = -self.base_omega_z * kff

        # Combined velocity
        joint1_vel = pid_output + feedforward
        joint1_vel = max(-max_vel, min(max_vel, joint1_vel))

        # Integrate to position
        new_pos = current_j1 + joint1_vel * self.dt
        new_pos = max(j1_min, min(j1_max, new_pos))

        self._publish_joint1(new_pos, joint1_vel)

    # ── Trajectory publishing ──────────────────────────────────

    def _publish_joint1(self, target_j1: float, velocity: float = 0.0):
        """Publish a JointTrajectory commanding ONLY joint1 (other joints untouched)."""
        traj = JointTrajectory()
        point = JointTrajectoryPoint()

        traj.joint_names.append(TARGET_JOINT)
        point.positions.append(target_j1)
        point.velocities.append(velocity)

        # Execute over two control periods for smoother interpolation
        period_ns = int(self.dt * 2e9)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = period_ns

        traj.points.append(point)
        self.traj_pub.publish(traj)


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
