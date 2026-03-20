#!/usr/bin/env python3

"""
Follow Face Node - Controls xArm to track a detected face using joint velocity commands.
Provides a /follow_face service to activate/deactivate face tracking,
and switches the arm between velocity mode (4) and MoveIt mode (1) accordingly.
"""

import time

import rclpy
from frida_constants.manipulation_constants import (
    FACE_RECOGNITION_LIFETIME,
    FOLLOW_FACE_SPEED,
    FOLLOW_FACE_TOLERANCE,
    MOVEIT_MODE,
)
from frida_interfaces.srv import FollowFace
from frida_motion_planning.utils.ros_utils import wait_for_future
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from utils.logger import Logger
from xarm_msgs.srv import GetDigitalIO, MoveVelocity, SetDigitalIO, SetInt16

XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity"
XARM_SETMODE_SERVICE = "/xarm/set_mode"
XARM_SETSTATE_SERVICE = "/xarm/set_state"

VELOCITY_MODE = 4
MAX_VELOCITY = 0.1
STOP_TIMEOUT = 1.0
SERVICE_TIMEOUT = 5.0
SET_MODE_RETRIES = 2
RUN_LOOP_PERIOD = 0.1


class FollowFaceNode(Node):
    """Node that tracks a detected face by sending joint velocity commands to the xArm."""

    def __init__(self):
        super().__init__("follow_face_node")
        callback_group = ReentrantCallbackGroup()

        # Face detection subscription
        self.create_subscription(
            Point, "/vision/follow_face", self._face_detection_callback, 2,
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
            Empty, "/manipulation/reset_xarm_controller", callback_group=callback_group
        )
        # TGPIO clients to preserve gripper state across mode switches
        self.get_tgpio_client = self.create_client(
            GetDigitalIO, "/xarm/get_tgpio_digital", callback_group=callback_group
        )
        self.set_tgpio_client = self.create_client(
            SetDigitalIO, "/xarm/set_tgpio_digital", callback_group=callback_group
        )

        # Wait for critical services
        if not self.move_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            Logger.warn(self, "Velocity move service not available")
        if not self.state_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            Logger.warn(self, "Set state service not available")
        if not self.mode_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            Logger.warn(self, "Set mode service not available")

        # Follow face service
        self.service = self.create_service(
            FollowFace, "/follow_face", self._follow_face_service_callback,
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

        # Previous velocities for stop detection
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.last_move_time = time.time()

        self.create_timer(RUN_LOOP_PERIOD, self._run_loop, callback_group=callback_group)
        self.get_logger().info("FollowFaceNode has started.")

    # -- Mode switching --

    def _read_tgpio_state(self):
        """Read current TGPIO digital values before mode switch."""
        try:
            future = self.get_tgpio_client.call_async(GetDigitalIO.Request())
            result = wait_for_future(future)
            if result and result.digitals:
                return list(result.digitals)
        except Exception as e:
            Logger.warn(self, f"Could not read TGPIO state: {e}")
        return None

    def _restore_tgpio_state(self, digitals):
        """Restore TGPIO digital values after mode switch."""
        if digitals is None:
            return
        try:
            for ionum, value in enumerate(digitals):
                req = SetDigitalIO.Request()
                req.ionum = ionum
                req.value = value
                future = self.set_tgpio_client.call_async(req)
                wait_for_future(future)
            Logger.info(self, f"Restored TGPIO state: {digitals}")
        except Exception as e:
            Logger.warn(self, f"Could not restore TGPIO state: {e}")

    def _set_xarm_mode(self, mode: int, reset_controller: bool = False) -> bool:
        """Set xArm mode and state. Preserves TGPIO (gripper) state across mode switch."""
        mode_request = SetInt16.Request()
        mode_request.data = mode
        state_request = SetInt16.Request()
        state_request.data = 0

        # Save gripper IO state before mode switch
        saved_tgpio = self._read_tgpio_state()

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

                # Restore gripper IO immediately after mode switch
                self._restore_tgpio_state(saved_tgpio)

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

    def _follow_face_service_callback(self, request: FollowFace.Request, response: FollowFace.Response):
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
        """Timer callback: send velocity commands to track the face."""
        if not self.is_following_face_active or not self.arm_ready:
            return

        x, y = self._get_face_position()

        if x is None or y is None:
            # No fresh face data — stop arm if it was previously moving
            if (self.prev_x != 0.0 or self.prev_y != 0.0) and \
               (time.time() - self.last_move_time) >= STOP_TIMEOUT:
                self._send_velocity(0.0, 0.0)
                self.prev_x = 0.0
                self.prev_y = 0.0
            return

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
