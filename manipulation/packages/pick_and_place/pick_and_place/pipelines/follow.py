import time
from dataclasses import dataclass, field

from frida_constants.manipulation_constants import (
    FACE_RECOGNITION_LIFETIME,
    FOLLOW_FACE_SPEED,
    FOLLOW_FACE_TOLERANCE,
    MOVEIT_MODE,
)

VELOCITY_MODE = 4
MAX_VELOCITY = 0.1
STOP_TIMEOUT = 1.0


# ======================================================================
# Face
# ======================================================================


@dataclass
class FaceState:
    """What the face node keeps between ticks."""

    # State
    is_following_face_active: bool = False
    arm_ready: bool = False

    # Face tracking data
    face_x: float = 0.0
    face_y: float = 0.0
    last_face_detection_time: float = 0.0
    has_new_face_data: bool = False

    # Previous velocities for stop detection
    prev_x: float = 0.0
    prev_y: float = 0.0
    last_move_time: float = field(default_factory=time.time)


def face_on(arm, state: FaceState):
    if state.is_following_face_active:
        arm.logger.info("Face following already active, skipping")
        return
    arm.logger.info("Activating face following")
    arm.set_mode(VELOCITY_MODE)
    time.sleep(0.5)
    state.arm_ready = True
    state.is_following_face_active = True


def face_off(arm, state: FaceState):
    if not state.is_following_face_active:
        arm.logger.info("Face following already inactive, skipping mode switch")
        return
    arm.logger.info("Deactivating face following")
    state.is_following_face_active = False
    state.arm_ready = False

    # Wait for current movement to finish
    timeout_start = time.time()
    while arm.busy and (time.time() - timeout_start) < 3.0:
        time.sleep(0.1)

    # Stop the arm
    _send_face_velocity(arm, 0.0, 0.0)
    time.sleep(1)

    # Switch back to MoveIt mode and reset controller
    arm.set_mode(MOVEIT_MODE, reset_controller=True)
    time.sleep(1)


def face_tick(arm, state: FaceState):
    """Timer callback: send velocity commands to track the face."""
    if not state.is_following_face_active or not state.arm_ready:
        return

    x, y = _get_face_position(arm, state)

    if x is None or y is None:
        # No fresh face data — stop arm if it was previously moving
        if (state.prev_x != 0.0 or state.prev_y != 0.0) and (
            time.time() - state.last_move_time
        ) >= STOP_TIMEOUT:
            _send_face_velocity(arm, 0.0, 0.0)
            state.prev_x = 0.0
            state.prev_y = 0.0
        return

    y = -y

    if abs(x) > FOLLOW_FACE_TOLERANCE or abs(y) > FOLLOW_FACE_TOLERANCE:
        _send_face_velocity(arm, x, y)
    else:
        _send_face_velocity(arm, 0.0, 0.0)

    state.prev_x = x
    state.prev_y = y
    state.last_move_time = time.time()


def face_speeds(x_vel: float, y_vel: float):
    """Clamps to MAX_VELOCITY and applies speed multiplier."""
    x_vel = max(-MAX_VELOCITY, min(MAX_VELOCITY, -x_vel)) * FOLLOW_FACE_SPEED
    y_vel = max(-MAX_VELOCITY, min(MAX_VELOCITY, y_vel)) * FOLLOW_FACE_SPEED
    return [x_vel, 0.0, 0.0, 0.0, y_vel, 0.0, 0.0]


def _get_face_position(arm, state: FaceState):
    """Return face position if fresh data is available, else (None, None)."""
    if not state.has_new_face_data:
        return None, None

    state.has_new_face_data = False

    if time.time() - state.last_face_detection_time > FACE_RECOGNITION_LIFETIME:
        arm.logger.warn("Face detection data is stale")
        return None, None

    return state.face_x, state.face_y


def _send_face_velocity(arm, x_vel: float, y_vel: float):
    """Send velocity command to xArm. Clamps to MAX_VELOCITY and applies speed multiplier."""
    if arm.busy:
        return

    speeds = face_speeds(x_vel, y_vel)

    try:
        arm.busy = True
        arm.send_joint_velocity(speeds)
    except Exception as e:
        arm.busy = False
        arm.logger.error(f"Error sending velocity command: {e}")
