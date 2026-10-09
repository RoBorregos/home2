import time
from dataclasses import dataclass, field

from frida_constants.manipulation_constants import (
    FACE_RECOGNITION_LIFETIME,
    FOLLOW_FACE_SPEED,
    FOLLOW_FACE_TOLERANCE,
)

MAX_VELOCITY = 0.1
STOP_TIMEOUT = 1.0
TARGET_JOINT = "joint1"


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
    arm.disable_tgpio_reset()
    arm.enter_joint_velocity_mode()
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
    while arm.joint_velocity_busy and (time.time() - timeout_start) < 3.0:
        time.sleep(0.1)

    # Stop the arm
    _send_face_velocity(arm, 0.0, 0.0)
    time.sleep(1)

    # Switch back to MoveIt mode
    arm.leave_joint_velocity_mode()
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
    if arm.joint_velocity_busy:
        return

    speeds = face_speeds(x_vel, y_vel)

    try:
        arm.joint_velocity_busy = True
        arm.send_joint_velocity(speeds)
    except Exception as e:
        arm.joint_velocity_busy = False
        arm.logger.error(f"Error sending velocity command: {e}")


@dataclass
class PersonState:
    active = False
    centroid_x = 0.0
    centroid_time = 0.0
    base_omega_z = 0.0
    joint_positions: dict = field(default_factory=dict)
    error_integral = 0.0
    # Centroid-rate derivative (filled by _centroid_cb, low-pass filtered).
    # Computing it in the 20 Hz control loop would alternate spike/zero
    # because the centroid arrives at its own rate.
    error_deriv = 0.0


@dataclass(frozen=True)
class PersonParams:
    kp: float
    ki: float
    kd: float
    kff: float
    dead_zone: float
    max_velocity: float
    joint1_min: float
    joint1_max: float
    soft_limit_margin: float
    centroid_timeout: float
    integral_clamp: float
    recenter_enabled: bool
    recenter_velocity: float
    base_yaw_enabled: bool
    base_yaw_kp: float
    joint1_neutral: float
    base_yaw_max: float


def person_centroid(arm, state: PersonState, msg):
    now = time.time()
    if state.centroid_time > 0.0:
        dt = now - state.centroid_time
        if 0.005 < dt < 0.5:
            d = (msg.x - state.centroid_x) / dt
            # LPF (~1/3 weight on the new sample) tames per-frame bbox jitter
            state.error_deriv = 0.35 * d + 0.65 * state.error_deriv
        elif dt >= 0.5:
            state.error_deriv = 0.0  # stale gap — a finite diff would spike
    state.centroid_x = msg.x
    state.centroid_time = now
    arm.logger.info(f"Centroid: {msg.x:.3f}", once=True)


def person_on(arm, state: PersonState):
    state.error_integral = 0.0
    state.error_deriv = 0.0
    state.centroid_time = 0.0  # don't act on a centroid from a past run
    arm.enter_joint_velocity_mode()
    # Activate only AFTER the mode switch: with the multithreaded
    # executor the control loop keeps ticking during the sleeps above,
    # and velocity commands before mode 4 error out on the xArm.
    state.active = True
    arm.logger.info("Following enabled (velocity mode)")


def person_off(arm, state: PersonState):
    state.active = False
    _send_joint_velocity(arm, 0.0)
    time.sleep(0.3)
    state.error_integral = 0.0
    state.error_deriv = 0.0
    arm.leave_joint_velocity_mode()
    arm.logger.info("Following disabled (back to MoveIt mode)")


# ── Control loop ───────────────────────────────────────────


def person_tick(arm, state: PersonState, params: PersonParams, dt) -> float:
    if not state.active:
        return 0.0

    if TARGET_JOINT not in state.joint_positions:
        return 0.0

    current_j1 = state.joint_positions[TARGET_JOINT]

    age = time.time() - state.centroid_time

    if state.centroid_time == 0.0 or age > params.centroid_timeout:
        state.error_integral = 0.0
        state.error_deriv = 0.0
        _recenter_or_stop(arm, params, current_j1)
        return 0.0

    # Error: positive centroid_x = person right = positive error
    # Sign is flipped in _send_joint_velocity (matching temp_follow.py)
    error = state.centroid_x

    # Dead zone (P/I only — the derivative keeps damping inside it)
    if abs(error) < params.dead_zone:
        error = 0.0

    # PID controller (derivative computed at centroid rate in _centroid_cb)
    state.error_integral += error * dt
    state.error_integral = max(
        -params.integral_clamp, min(params.integral_clamp, state.error_integral)
    )

    pid_output = (
        params.kp * error
        + params.ki * state.error_integral
        + params.kd * state.error_deriv
    )

    # Feedforward: compensate base rotation
    feedforward = state.base_omega_z * params.kff

    # Combined velocity
    joint1_vel = pid_output + feedforward
    joint1_vel = max(-params.max_velocity, min(params.max_velocity, joint1_vel))

    # Soft limits: taper to zero across soft_limit_margin instead of a hard
    # cutoff, so the wide joint1 range never ends in a full-speed slam
    # (that slam was the old xArm fault mode).
    joint1_vel, limited = _apply_soft_limit(params, current_j1, joint1_vel)
    if limited:
        state.error_integral = 0.0

    arm.logger.info(
        f"j1={current_j1:.3f} cx={state.centroid_x:.3f} "
        f"pid={pid_output:.3f} ff={feedforward:.3f} vel={joint1_vel:.3f}",
        throttle_duration_sec=0.5,
    )
    _send_joint_velocity(arm, joint1_vel)

    # Reactive base yaw: rotate the base to bring joint1 back toward neutral
    # (unload the arm) so the person stays centred with unlimited pan range.
    if params.base_yaw_enabled:
        base_yaw = params.base_yaw_kp * (current_j1 - params.joint1_neutral)
        base_yaw = max(-params.base_yaw_max, min(params.base_yaw_max, base_yaw))
    else:
        base_yaw = 0.0
    return base_yaw


def _apply_soft_limit(params: PersonParams, current_j1: float, cmd_vel: float):
    """Taper the commanded velocity to zero across soft_limit_margin before a
    joint1 limit. Returns (scaled_vel, was_limited). Sign note: the actual
    joint velocity is -cmd_vel (negated in _send_joint_velocity), so
    cmd_vel > 0 moves joint1 NEGATIVE (toward joint1_min)."""
    if cmd_vel == 0.0:
        return 0.0, False
    if cmd_vel > 0:  # actual motion toward j1_min
        dist = current_j1 - params.joint1_min
    else:  # actual motion toward j1_max
        dist = params.joint1_max - current_j1
    if dist >= params.soft_limit_margin:
        return cmd_vel, False
    scale = max(0.0, dist / params.soft_limit_margin)
    return cmd_vel * scale, True


def _recenter_or_stop(arm, params: PersonParams, current_j1: float):
    """No centroid: either hold still (legacy) or pan slowly back to
    joint1_neutral so the camera faces where the base is heading (the
    smoother drives to the person's last-known position on loss)."""
    if not params.recenter_enabled:
        _send_joint_velocity(arm, 0.0)
        arm.logger.warn(
            "No centroid data — sending zero velocity", throttle_duration_sec=2.0
        )
        return
    neutral = params.joint1_neutral
    err = neutral - current_j1  # desired actual joint displacement
    if abs(err) < 0.05:
        _send_joint_velocity(arm, 0.0)
        return
    actual_vel = max(-params.recenter_velocity, min(params.recenter_velocity, err))
    # command is negated by _send_joint_velocity => pass -actual_vel
    _send_joint_velocity(arm, -actual_vel)
    arm.logger.warn(
        f"No centroid data — recentering joint1 ({current_j1:.2f} -> {neutral:.2f})",
        throttle_duration_sec=2.0,
    )

    # ── Velocity command ───────────────────────────────────────


def _send_joint_velocity(arm, velocity: float):
    """Send joint velocity command — only joint1, all others 0."""
    try:
        arm.send_joint_velocity([-velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    except Exception as e:
        arm.logger.error(f"Velocity command failed: {e}")
