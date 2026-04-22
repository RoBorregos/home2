#!/usr/bin/env python3

"""
Follow Face Node - Controls xArm to track a detected face using joint velocity commands.
Provides a /follow_face service to activate/deactivate face tracking,
and switches the arm between velocity mode (4) and MoveIt mode (1) accordingly.
"""

import time

import rclpy
from frida_constants.hri_constants import DOA_OFFSET, RESPEAKER_DOA_TOPIC
from frida_constants.manipulation_constants import (
    FACE_RECOGNITION_LIFETIME,
    FOLLOW_FACE_SPEED,
    FOLLOW_FACE_TOLERANCE,
    MOVEIT_MODE,
)
from frida_interfaces.srv import FollowFace
from frida_motion_planning.utils.ros_utils import wait_for_future
from geometry_msgs.msg import Point, TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Int16
from std_srvs.srv import Empty
from task_manager.utils.logger import Logger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from xarm_msgs.srv import MoveVelocity, SetInt16

XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"

VELOCITY_MODE = 4
MAX_VELOCITY = 0.1
STOP_TIMEOUT = 1.0
SERVICE_TIMEOUT = 5.0
SET_MODE_RETRIES = 2
RUN_LOOP_PERIOD = 0.1

# DOA-based fallback: when vision loses the face, use the DOA angle to rotate toward the speaker
DOA_FALLBACK_TIMEOUT = 0.5  # seconds after losing vision before using DOA
DOA_LIFETIME = 1.0  # max age of a DOA reading to consider it valid
DOA_SPEED_FACTOR = 0.5  # slower than vision-based tracking
DOA_DEAD_ZONE = 15.0  # degrees — don't move if speaker is roughly in front

# Offset between respeaker and xarm base (meters).
# The respeaker sits ~0.5m to the right of the xarm base (negative Y in link_base frame).
RESPEAKER_OFFSET_Y = -0.5


class FollowFaceNode(Node):
    """Node that tracks a detected face by sending joint velocity commands to the xArm."""

    def __init__(self):
        super().__init__("follow_face_node")
        callback_group = ReentrantCallbackGroup()

        # Face detection subscription
        self.create_subscription(
            Point,
            "/vision/follow_face",
            self._face_detection_callback,
            2,
            callback_group=callback_group,
        )

        # DOA subscription (raw respeaker topic) — used as fallback when vision loses the face
        self.create_subscription(
            Int16,
            RESPEAKER_DOA_TOPIC,
            self._doa_callback,
            10,
            callback_group=callback_group,
        )

        # Publish static TF: link_base -> respeaker_link
        self._publish_respeaker_tf()

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
            Empty, "/manipulation/reset_xarm_controller", callback_group=callback_group
        )
        # Client to configure the xArm driver to NOT reset TGPIO outputs
        # when the robot state/mode changes. Without this, switching between
        # MoveIt mode (1) and velocity mode (4) resets the gripper (opens it).
        self.config_tgpio_reset_client = self.create_client(
            SetInt16, "/xarm/config_tgpio_reset_when_stop", callback_group=callback_group
        )

        # Wait for critical services
        if not self.move_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            Logger.warn(self, "Velocity move service not available")
        if not self.state_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            Logger.warn(self, "Set state service not available")
        if not self.mode_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            Logger.warn(self, "Set mode service not available")

        # Disable TGPIO reset on state changes so the gripper stays closed
        # across mode switches. Must be called AFTER the driver is up.
        if self.config_tgpio_reset_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            req = SetInt16.Request()
            req.data = 0
            future = self.config_tgpio_reset_client.call_async(req)
            wait_for_future(future)
            Logger.info(
                self, "TGPIO reset on stop disabled (gripper preserved across mode switches)"
            )
        else:
            Logger.warn(
                self,
                "config_tgpio_reset_when_stop service not available -- gripper may open during mode switches",
            )

        # Follow face service
        self.service = self.create_service(
            FollowFace,
            "/follow_face",
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

        # DOA tracking data (fallback when no vision)
        self.doa_angle = 0.0  # remapped angle: 0=front, negative=left, positive=right
        self.last_doa_time = 0.0

        # Previous velocities for stop detection
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.last_move_time = time.time()

        self.create_timer(RUN_LOOP_PERIOD, self._run_loop, callback_group=callback_group)
        self.get_logger().info("FollowFaceNode has started.")

    # -- Static TF --

    def _publish_respeaker_tf(self):
        """Publish a static transform from link_base to respeaker_link.

        This lets any node look up the spatial relationship between the xarm
        base and the respeaker microphone array via TF2.
        """
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "link_base"
        t.child_frame_id = "respeaker_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = RESPEAKER_OFFSET_Y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)
        Logger.info(
            self,
            f"Published static TF: link_base -> respeaker_link (offset y={RESPEAKER_OFFSET_Y}m)",
        )

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
                Logger.info(self, f"Setting mode to {mode} (attempt {attempt + 1})")
                future_mode = self.mode_client.call_async(mode_request)
                future_mode = wait_for_future(future_mode)
                if not future_mode:
                    Logger.error(self, "Failed to set mode")
                    continue
                Logger.success(self, "Mode set")

                Logger.info(self, "Setting state to 0 (active)")
                future_state = self.state_client.call_async(state_request)
                future_state = wait_for_future(future_state)
                if not future_state:
                    Logger.error(self, "Failed to set state")
                    continue
                Logger.success(self, "State set")

                if reset_controller:
                    Logger.info(self, "Resetting trajectory controller")
                    future_ctrl = self.reset_controller_client.call_async(Empty.Request())
                    future_ctrl = wait_for_future(future_ctrl)
                    if not future_ctrl:
                        Logger.error(self, "Failed to reset controller")
                        continue
                    Logger.success(self, "Controller reset successfully")

                return True
            except Exception as e:
                Logger.error(self, f"Error setting arm mode: {e}")

        Logger.error(self, f"Failed to set mode {mode} after {SET_MODE_RETRIES} attempts")
        return False

    # -- Service callback --

    def _follow_face_service_callback(
        self, request: FollowFace.Request, response: FollowFace.Response
    ):
        """Handle follow face service requests."""
        if request.follow_face:
            if self.is_following_face_active:
                Logger.info(self, "Face following already active, skipping")
                response.success = True
                return response
            Logger.info(self, "Activating face following")
            self._set_xarm_mode(VELOCITY_MODE)
            time.sleep(0.5)
            self.arm_ready = True
            self.is_following_face_active = True
        else:
            if not self.is_following_face_active:
                Logger.info(self, "Face following already inactive, skipping mode switch")
                response.success = True
                return response
            Logger.info(self, "Deactivating face following")
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
            Logger.warn(self, "Face detection data is stale")
            return None, None

        return self.face_x, self.face_y

    # -- DOA (Direction of Arrival) --

    def _doa_callback(self, msg: Int16):
        """Receive raw DOA angle from respeaker and remap to robot frame.

        Raw ReSpeaker: 0°=MIC1 (right side), clockwise 0-359.
        After DOA_OFFSET: 0°=front, negative=left, positive=right (-180 to +180).
        """
        raw = msg.data
        remapped = (raw - DOA_OFFSET) % 360
        if remapped > 180:
            remapped -= 360
        self.doa_angle = remapped
        self.last_doa_time = time.time()

    def _get_doa_velocity(self):
        """Convert DOA angle to an x velocity for joint1 rotation.

        Returns x_vel or None if DOA data is stale or speaker is in the dead zone.
        """
        if time.time() - self.last_doa_time > DOA_LIFETIME:
            return None

        if abs(self.doa_angle) < DOA_DEAD_ZONE:
            return None  # speaker is roughly in front, no need to rotate

        # Normalize angle to [-1, 1] range (180° = full speed)
        x_vel = self.doa_angle / 180.0
        x_vel = max(-1.0, min(1.0, x_vel)) * DOA_SPEED_FACTOR
        return x_vel

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
            Logger.error(self, f"Error sending velocity command: {e}")

    def _velocity_done_callback(self, future):
        """Callback when velocity command completes."""
        try:
            result = future.result()
            if not result:
                Logger.error(self, "Velocity command returned no result")
        except Exception as e:
            Logger.error(self, f"Velocity command failed: {e}")
        finally:
            self.arm_moving = False

    # -- Main loop --

    def _run_loop(self):
        """Timer callback: send velocity commands to track the face.

        Priority: vision (face position) > DOA (speaker direction).
        When vision is available, use it directly.
        When vision is lost for DOA_FALLBACK_TIMEOUT seconds, rotate toward the
        speaker's DOA angle so the camera can reacquire the face.
        """
        if not self.is_following_face_active or not self.arm_ready:
            return

        x, y = self._get_face_position()

        if x is not None and y is not None:
            # Vision available — use it
            y = -y
            if abs(x) > FOLLOW_FACE_TOLERANCE or abs(y) > FOLLOW_FACE_TOLERANCE:
                self._send_velocity(x, y)
            else:
                self._send_velocity(0.0, 0.0)
            self.prev_x = x
            self.prev_y = y
            self.last_move_time = time.time()
            return

        # No vision — check if we should fall back to DOA
        time_since_last_face = time.time() - self.last_face_detection_time
        if time_since_last_face > DOA_FALLBACK_TIMEOUT:
            doa_vel = self._get_doa_velocity()
            if doa_vel is not None:
                self._send_velocity(doa_vel, 0.0)
                self.prev_x = doa_vel
                self.prev_y = 0.0
                self.last_move_time = time.time()
                return

        # Neither vision nor DOA — stop if previously moving
        if (self.prev_x != 0.0 or self.prev_y != 0.0) and (
            time.time() - self.last_move_time
        ) >= STOP_TIMEOUT:
            self._send_velocity(0.0, 0.0)
            self.prev_x = 0.0
            self.prev_y = 0.0


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    node = FollowFaceNode()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
