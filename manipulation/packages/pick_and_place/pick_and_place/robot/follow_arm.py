"""FollowArm: the xArm as the follow nodes see it.

The only owner of the /xarm clients that follow_face and
follow_person_controller use; the follow pipeline talks only to this class.
"""

import time

from frida_constants.manipulation_constants import (
    MANIPULATION_ENSURE_ARM_READY_SERVICE,
    XARM_MOVEVELOCITY_SERVICE,
    XARM_SETMODE_SERVICE,
    XARM_SETSTATE_SERVICE,
)
from frida_motion_planning.utils.ros_utils import wait_for_future
from std_srvs.srv import Trigger
from xarm_msgs.srv import MoveVelocity, SetInt16

SERVICE_TIMEOUT = 5.0
SET_MODE_RETRIES = 2


class FollowArm:
    """The xArm, as the follow nodes see it."""

    def __init__(self, node, callback_group, on_velocity_done, face_extras=False):
        self.logger = node.get_logger()
        # True while a velocity command is in flight (the face node sends one at a time)
        self.busy = False
        self._on_velocity_done = on_velocity_done

        # Service clients
        self._mode_client = node.create_client(
            SetInt16, XARM_SETMODE_SERVICE, callback_group=callback_group
        )
        self._state_client = node.create_client(
            SetInt16, XARM_SETSTATE_SERVICE, callback_group=callback_group
        )
        self._move_client = node.create_client(
            MoveVelocity, XARM_MOVEVELOCITY_SERVICE, callback_group=callback_group
        )
        if face_extras:
            self._reset_controller_client = node.create_client(
                Trigger,
                MANIPULATION_ENSURE_ARM_READY_SERVICE,
                callback_group=callback_group,
            )
            # Client to configure the xArm driver to NOT reset TGPIO outputs
            # when the robot state/mode changes. Without this, switching between
            # MoveIt mode (1) and velocity mode (4) resets the gripper (opens it).
            self._config_tgpio_reset_client = node.create_client(
                SetInt16,
                "/xarm/config_tgpio_reset_when_stop",
                callback_group=callback_group,
            )

    # -- Face node startup --

    def wait_for_services(self):
        # Wait for critical services
        if not self._move_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.logger.warn("Velocity move service not available")
        if not self._state_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.logger.warn("Set state service not available")
        if not self._mode_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.logger.warn("Set mode service not available")

    def disable_tgpio_reset(self):
        # Disable TGPIO reset on state changes so the gripper stays closed
        # across mode switches. Must be called AFTER the driver is up.
        if self._config_tgpio_reset_client.wait_for_service(
            timeout_sec=SERVICE_TIMEOUT
        ):
            req = SetInt16.Request()
            req.data = 0
            future = self._config_tgpio_reset_client.call_async(req)
            wait_for_future(future)
            self.logger.info(
                "TGPIO reset on stop disabled (gripper preserved across mode switches)"
            )
        else:
            self.logger.warn(
                "config_tgpio_reset_when_stop service not available -- gripper may open during mode switches",
            )

    # -- Mode switching --

    def set_mode(self, mode: int, reset_controller: bool = False) -> bool:
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
                self.logger.info(f"Setting mode to {mode} (attempt {attempt + 1})")
                future_mode = self._mode_client.call_async(mode_request)
                future_mode = wait_for_future(future_mode)
                if not future_mode:
                    self.logger.error("Failed to set mode")
                    continue
                self.logger.info("Mode set")

                self.logger.info("Setting state to 0 (active)")
                future_state = self._state_client.call_async(state_request)
                future_state = wait_for_future(future_state)
                if not future_state:
                    self.logger.error("Failed to set state")
                    continue
                self.logger.info("State set")

                if reset_controller:
                    self.logger.info("Resetting trajectory controller")
                    future_ctrl = self._reset_controller_client.call_async(
                        Trigger.Request()
                    )
                    future_ctrl = wait_for_future(future_ctrl)
                    if not future_ctrl:
                        self.logger.error("Failed to reset controller")
                        continue
                    self.logger.info("Controller reset successfully")

                return True
            except Exception as e:
                self.logger.error(f"Error setting arm mode: {e}")

        self.logger.error(
            f"Failed to set mode {mode} after {SET_MODE_RETRIES} attempts"
        )
        return False

    def set_mode_no_wait(self, mode: int):
        """Set xArm mode + state 0. Must set mode first, then state."""
        try:
            # Step 1: clear errors by setting state 0
            state_req = SetInt16.Request()
            state_req.data = 0
            self._state_client.call_async(state_req)
            time.sleep(0.5)

            # Step 2: set desired mode
            mode_req = SetInt16.Request()
            mode_req.data = mode
            self._mode_client.call_async(mode_req)
            time.sleep(0.5)

            # Step 3: set state 0 again to activate
            self._state_client.call_async(state_req)
            time.sleep(0.5)

            self.logger.info(f"Arm mode set to {mode}")
        except Exception as e:
            self.logger.error(f"Failed to set arm mode: {e}")

    # -- Movement --

    def send_joint_velocity(self, speeds):
        """Send one joint velocity command; the node's done callback reports it."""
        req = MoveVelocity.Request()
        req.is_sync = True
        req.speeds = speeds
        future = self._move_client.call_async(req)
        future.add_done_callback(self._on_velocity_done)
