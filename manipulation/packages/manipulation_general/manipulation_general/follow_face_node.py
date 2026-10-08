#!/usr/bin/env python3

"""
Follow Face Node - Controls xArm to track a detected face using joint velocity commands.
Provides a /follow_face service to activate/deactivate face tracking,
and switches the arm between velocity mode (4) and MoveIt mode (1) accordingly.

When no face has been seen for a while, it falls back to the ReSpeaker DOA
(direction of arrival): if someone spoke recently, joint1 turns toward the
voice so the camera can find their face, and face tracking takes over again.
The mic is fixed next to the arm (it does not turn with joint1), so the DOA is
relative to the robot's front. Calibrate doa_offset_deg / doa_sign on the robot
with --log-level follow_face_node:=debug.
"""

import math
import time

import rclpy
from frida_constants.hri_constants import RESPEAKER_DOA_TOPIC, VOICE_ACTIVITY_TOPIC
from frida_constants.manipulation_constants import (
    FACE_RECOGNITION_LIFETIME,
    FOLLOW_FACE_SPEED,
    FOLLOW_FACE_TOLERANCE,
    MOVEIT_MODE,
    MANIPULATION_ENSURE_ARM_READY_SERVICE,
    FOLLOW_FACE_ARM_SERVICE,
)
from frida_constants.vision_constants import FOLLOW_TOPIC
from frida_interfaces.srv import FollowFace
from frida_motion_planning.utils.ros_utils import wait_for_future
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int16
from std_srvs.srv import Trigger
from xarm_msgs.srv import MoveVelocity, SetInt16

SAYING_TOPIC = "/saying"
JOINT_STATES_TOPIC = "/joint_states"
PAN_JOINT = "joint1"

XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"

VELOCITY_MODE = 4
MAX_VELOCITY = 0.1
STOP_TIMEOUT = 1.0
SERVICE_TIMEOUT = 5.0
SET_MODE_RETRIES = 2
RUN_LOOP_PERIOD = 0.1


def wrap_deg(angle: float) -> float:
    """Wrap an angle in degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def doa_to_joint1_target(
    doa_deg: float,
    offset_deg: float,
    sign: float,
    neutral: float,
    joint_min: float,
    joint_max: float,
) -> float:
    """joint1 angle (rad) that points the camera at a DOA reading.

    offset_deg is the DOA read when the speaker is straight ahead (joint1 at
    neutral); sign flips the direction if the mic's angle grows opposite to
    joint1. Voices outside the reachable range clamp to the nearest limit.
    """
    error = wrap_deg(doa_deg - offset_deg)
    target = neutral + sign * math.radians(error)
    return min(max(target, joint_min), joint_max)


class FollowFaceNode(Node):
    """Node that tracks a detected face by sending joint velocity commands to the xArm."""

    def __init__(self):
        super().__init__("follow_face_node")
        callback_group = ReentrantCallbackGroup()

        # DOA fallback parameters (see module docstring)
        self.declare_parameter("doa_enabled", True)
        # DOA reading when the speaker is straight ahead of the robot.
        self.declare_parameter("doa_offset_deg", 0.0)
        # +1 / -1: flip if the arm turns away from the voice.
        self.declare_parameter("doa_sign", 1.0)
        # joint1 pointing forward and its usable range (as follow_person_controller).
        self.declare_parameter("joint1_neutral", -1.5707)
        self.declare_parameter("joint1_min", -3.05)
        self.declare_parameter("joint1_max", -0.2)
        # Seconds without a face before turning to the voice. Longer than
        # FACE_RECOGNITION_LIFETIME so a dropped frame doesn't trigger it.
        self.declare_parameter("face_lost_timeout", 1.0)
        # Only trust the DOA if voice was detected within this many seconds.
        self.declare_parameter("voice_recent_s", 1.5)
        self.declare_parameter("doa_kp", 1.5)
        self.declare_parameter("doa_max_vel", 0.5)  # rad/s
        self.declare_parameter("doa_tolerance_deg", 8.0)

        # Face detection subscription
        self.create_subscription(
            Point,
            FOLLOW_TOPIC,
            self._face_detection_callback,
            2,
            callback_group=callback_group,
        )

        # DOA fallback inputs
        self.create_subscription(
            Int16,
            RESPEAKER_DOA_TOPIC,
            self._doa_callback,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            Bool,
            VOICE_ACTIVITY_TOPIC,
            self._voice_activity_callback,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            Bool, SAYING_TOPIC, self._saying_callback, 10, callback_group=callback_group
        )
        self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._joint_states_callback,
            10,
            callback_group=callback_group,
        )

        # Service clients
        self.mode_client = self.create_client(
            SetInt16, XARM_SETMODE_SERVICE, callback_group=callback_group
        )
        self.state_client = self.create_client(
            SetInt16, XARM_SETSTATE_SERVICE, callback_group=callback_group
        )
        self.move_client = self.create_client(
            MoveVelocity, XARM_MOVEVELOCITY_SERVICE, callback_group=callback_group
        )
        self.reset_controller_client = self.create_client(
            Trigger,
            MANIPULATION_ENSURE_ARM_READY_SERVICE,
            callback_group=callback_group,
        )
        # Client to configure the xArm driver to NOT reset TGPIO outputs
        # when the robot state/mode changes. Without this, switching between
        # MoveIt mode (1) and velocity mode (4) resets the gripper (opens it).
        self.config_tgpio_reset_client = self.create_client(
            SetInt16,
            "/xarm/config_tgpio_reset_when_stop",
            callback_group=callback_group,
        )

        # Wait for critical services
        if not self.move_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().warn("Velocity move service not available")
        if not self.state_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().warn("Set state service not available")
        if not self.mode_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().warn("Set mode service not available")

        # Disable TGPIO reset on state changes so the gripper stays closed
        # across mode switches. Must be called AFTER the driver is up.
        if self.config_tgpio_reset_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            req = SetInt16.Request()
            req.data = 0
            future = self.config_tgpio_reset_client.call_async(req)
            wait_for_future(future)
            self.get_logger().info(
                "TGPIO reset on stop disabled (gripper preserved across mode switches)"
            )
        else:
            self.get_logger().warn(
                "config_tgpio_reset_when_stop service not available -- gripper may open during mode switches",
            )

        # Follow face service
        self.service = self.create_service(
            FollowFace,
            FOLLOW_FACE_ARM_SERVICE,
            self._follow_face_service_callback,
            callback_group=callback_group,
        )

        # State
        self.is_following_face_active = False
        self.arm_ready = False
        self.arm_moving = False

        # Face tracking data
        self.face_x = 0.0
        self.face_y = 0.0
        self.last_face_detection_time = 0.0
        self.has_new_face_data = False

        # DOA fallback data
        self.doa_deg = None
        self.last_voice_time = 0.0
        self.robot_speaking = False
        self.joint1 = None
        self.last_face_seen = 0.0
        self.doa_turning = False

        # Previous velocities for stop detection
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.last_move_time = time.time()

        self.create_timer(
            RUN_LOOP_PERIOD, self._run_loop, callback_group=callback_group
        )
        self.get_logger().info("FollowFaceNode has started.")

    # -- Mode switching --

    def _set_xarm_mode(self, mode: int, reset_controller: bool = False) -> bool:
        """Set xArm mode and state.

        Gripper state is preserved automatically thanks to the
        config_tgpio_reset_when_stop(0) call done at init.
        """
        mode_request = SetInt16.Request()
        mode_request.data = mode
        state_request = SetInt16.Request()
        state_request.data = 0

        for attempt in range(SET_MODE_RETRIES):
            try:
                self.get_logger().info(
                    f"Setting mode to {mode} (attempt {attempt + 1})"
                )
                future_mode = self.mode_client.call_async(mode_request)
                future_mode = wait_for_future(future_mode)
                if not future_mode:
                    self.get_logger().error("Failed to set mode")
                    continue
                self.get_logger().info("Mode set")

                self.get_logger().info("Setting state to 0 (active)")
                future_state = self.state_client.call_async(state_request)
                future_state = wait_for_future(future_state)
                if not future_state:
                    self.get_logger().error("Failed to set state")
                    continue
                self.get_logger().info("State set")

                if reset_controller:
                    self.get_logger().info("Resetting trajectory controller")
                    future_ctrl = self.reset_controller_client.call_async(
                        Trigger.Request()
                    )
                    future_ctrl = wait_for_future(future_ctrl)
                    if not future_ctrl:
                        self.get_logger().error("Failed to reset controller")
                        continue
                    self.get_logger().info("Controller reset successfully")

                return True
            except Exception as e:
                self.get_logger().error(f"Error setting arm mode: {e}")

        self.get_logger().error(
            f"Failed to set mode {mode} after {SET_MODE_RETRIES} attempts"
        )
        return False

    # -- Service callback --

    def _follow_face_service_callback(
        self, request: FollowFace.Request, response: FollowFace.Response
    ):
        """Handle follow face service requests."""
        if request.follow_face:
            if self.is_following_face_active:
                self.get_logger().info("Face following already active, skipping")
                response.success = True
                return response
            self.get_logger().info("Activating face following")
            self._set_xarm_mode(VELOCITY_MODE)
            time.sleep(0.5)
            # Give the camera face_lost_timeout to find a face before the DOA
            # fallback may move the arm.
            self.last_face_seen = time.time()
            self.arm_ready = True
            self.is_following_face_active = True
        else:
            if not self.is_following_face_active:
                self.get_logger().info(
                    "Face following already inactive, skipping mode switch"
                )
                response.success = True
                return response
            self.get_logger().info("Deactivating face following")
            self.is_following_face_active = False
            self.arm_ready = False

            # Wait for current movement to finish
            timeout_start = time.time()
            while self.arm_moving and (time.time() - timeout_start) < 3.0:
                time.sleep(0.1)

            # Stop the arm
            self._send_velocity(0.0, 0.0)
            time.sleep(1)

            # Switch back to MoveIt mode and reset controller
            self._set_xarm_mode(MOVEIT_MODE, reset_controller=True)
            time.sleep(1)

        response.success = True
        return response

    # -- Face detection --

    def _face_detection_callback(self, msg: Point):
        """Receive face position from vision."""
        self.face_x = msg.x
        self.face_y = msg.y
        self.last_face_detection_time = time.time()
        self.has_new_face_data = True

    def _get_face_position(self):
        """Return face position if fresh data is available, else (None, None)."""
        if not self.has_new_face_data:
            return None, None

        self.has_new_face_data = False

        if time.time() - self.last_face_detection_time > FACE_RECOGNITION_LIFETIME:
            self.get_logger().warn("Face detection data is stale")
            return None, None

        return self.face_x, self.face_y

    # -- DOA fallback inputs --

    def _doa_callback(self, msg: Int16):
        self.doa_deg = float(msg.data)

    def _voice_activity_callback(self, msg: Bool):
        if msg.data:
            self.last_voice_time = time.time()

    def _saying_callback(self, msg: Bool):
        self.robot_speaking = msg.data

    def _joint_states_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == PAN_JOINT:
                self.joint1 = pos
                return

    # -- Movement --

    def _send_velocity(self, x_vel: float, y_vel: float):
        """Send velocity command to xArm. Clamps to MAX_VELOCITY and applies speed multiplier."""
        if self.arm_moving:
            return

        x_vel = max(-MAX_VELOCITY, min(MAX_VELOCITY, -x_vel)) * FOLLOW_FACE_SPEED
        y_vel = max(-MAX_VELOCITY, min(MAX_VELOCITY, y_vel)) * FOLLOW_FACE_SPEED

        motion_msg = MoveVelocity.Request()
        motion_msg.is_sync = True
        motion_msg.speeds = [x_vel, 0.0, 0.0, 0.0, y_vel, 0.0, 0.0]

        try:
            self.arm_moving = True
            future = self.move_client.call_async(motion_msg)
            future.add_done_callback(self._velocity_done_callback)
        except Exception as e:
            self.arm_moving = False
            self.get_logger().error(f"Error sending velocity command: {e}")

    def _send_joint1_velocity(self, velocity: float) -> bool:
        """Send a raw joint1 velocity (rad/s); the face scaling/sign don't apply.

        Returns False if skipped because a previous command is still in flight.
        """
        if self.arm_moving:
            return False

        motion_msg = MoveVelocity.Request()
        motion_msg.is_sync = True
        motion_msg.speeds = [velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        try:
            self.arm_moving = True
            future = self.move_client.call_async(motion_msg)
            future.add_done_callback(self._velocity_done_callback)
            return True
        except Exception as e:
            self.arm_moving = False
            self.get_logger().error(f"Error sending velocity command: {e}")
            return False

    def _turn_to_voice(self) -> bool:
        """Turn joint1 toward the last DOA if someone spoke recently.

        Returns True if the DOA fallback is in control this tick.
        """
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        now = time.time()
        if (
            not p("doa_enabled")
            or now - self.last_face_seen < p("face_lost_timeout")
            or now - self.last_voice_time > p("voice_recent_s")
            or self.robot_speaking
        ):
            return False
        if self.doa_deg is None or self.joint1 is None:
            self.get_logger().warn(
                "DOA fallback idle: no DOA (ReSpeaker connected?) or no joint states",
                once=True,
            )
            return False

        target = doa_to_joint1_target(
            self.doa_deg,
            p("doa_offset_deg"),
            p("doa_sign"),
            p("joint1_neutral"),
            p("joint1_min"),
            p("joint1_max"),
        )
        error = target - self.joint1
        self.get_logger().debug(
            f"DOA raw={self.doa_deg:.0f} deg | rel={wrap_deg(self.doa_deg - p('doa_offset_deg')):.0f} deg"
            f" | joint1={self.joint1:.2f} -> target={target:.2f} rad",
            throttle_duration_sec=1.0,
        )

        if abs(error) > math.radians(p("doa_tolerance_deg")):
            max_vel = p("doa_max_vel")
            self._send_joint1_velocity(max(-max_vel, min(max_vel, p("doa_kp") * error)))
            self.doa_turning = True
        elif self.doa_turning and self._send_joint1_velocity(0.0):
            self.doa_turning = False
        return True

    def _stop_doa_turn(self):
        """xArm velocity mode keeps the last command: stop at once when the DOA
        fallback hands over, instead of waiting STOP_TIMEOUT and overshooting."""
        if self.doa_turning and self._send_joint1_velocity(0.0):
            self.doa_turning = False

    def _velocity_done_callback(self, future):
        """Callback when velocity command completes."""
        try:
            result = future.result()
            if not result:
                self.get_logger().error("Velocity command returned no result")
        except Exception as e:
            self.get_logger().error(f"Velocity command failed: {e}")
        finally:
            self.arm_moving = False

    # -- Main loop --

    def _run_loop(self):
        """Timer callback: send velocity commands to track the face."""
        if not self.is_following_face_active or not self.arm_ready:
            return

        x, y = self._get_face_position()

        if x is None or y is None:
            # No face for a while — turn toward whoever is speaking, if anyone.
            if self._turn_to_voice():
                return
            self._stop_doa_turn()
            # No fresh face data — stop arm if it was previously moving
            if (self.prev_x != 0.0 or self.prev_y != 0.0) and (
                time.time() - self.last_move_time
            ) >= STOP_TIMEOUT:
                self._send_velocity(0.0, 0.0)
                self.prev_x = 0.0
                self.prev_y = 0.0
            return

        self.last_face_seen = time.time()
        self.doa_turning = False  # the face command below supersedes the turn
        y = -y

        if abs(x) > FOLLOW_FACE_TOLERANCE or abs(y) > FOLLOW_FACE_TOLERANCE:
            self._send_velocity(x, y)
        else:
            self._send_velocity(0.0, 0.0)

        self.prev_x = x
        self.prev_y = y
        self.last_move_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    node = FollowFaceNode()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
