"""
odrive_dashboard_node.py
========================
ROS2 node with an optional embedded web dashboard at http://localhost:5000.

New parameter:
  enable_web_gui  (bool, default False)
  web_gui_port    (int,  default 5000)

When True, a Flask + Flask-SocketIO server starts in a background thread.
The dashboard mirrors gui.odriverobotics.com/dashboard:
  - Live telemetry for all 4 axes (state, pos, vel, voltage, current, errors)
  - Per-axis arm / IDLE / calibrate / reboot buttons
  - Full startup sequence (global or per-axis)
  - Clear errors, stop all
  - Velocity / current limit sliders
  - PID gain sliders (pos_gain, vel_gain, vel_int_gain)
  - Controller mode selector
  - Axis target bitmask selector for config commands
  - Manual velocity override sliders
  - Live Vx/Vy/Wz velocity chart
  - Activity log panel
  - Firmware error log panel (separate from the activity log)
  - Axis Lab tab (per-motor inspection/control/calibration -- UNTESTED,
    not yet validated against real hardware, see status.txt)

This package mirrors home-custom-base's `odrive_comm` package (same dashboard,
hardened for production use on the home2 robot). For full design detail on
the on-MCU EKF, the firmware error-reporting pipeline, and the CAN parameter
(RxSdo) feature, see:
  https://github.com/RoBorregos/home-custom-base/tree/main
  firmware/STM32H7_OMNIBASE_CAN_BNO085/omnibase_documentation.md (sections
  7, 10, 11 specifically)

Install dependencies (only needed when enable_web_gui=True):
    pip install flask flask-socketio

Serial Protocol (PC -> STM32)
------------------------------
  Type 1 – Control:  "1 <vx> <vy> <wz>\r\n"
  Type 2 – Config:   "2 <sub_type> <mask_hex> [params...]\r\n"

    sub_type  20 clear_errors
              21 set_state         <state>
              22 set_ctrl_mode     <ctrl_mode> <input_mode>
              23 set_limits        <vel_lim> <curr_lim>
              24 set_pos_gain      <pos_gain>
              25 set_vel_gains     <vel_gain> <vel_int_gain>
              26 startup           <ctrl_mode> <input_mode> <state>
              27 reboot
              28 set_torque        <torque>
              29 stop
              30 set_input_pos     <pos> <vel_ff> <torque_ff>
              31 set_param_float   <endpoint_id> <value>
                    Arbitrary ODrive parameter write over CAN (RxSdo,
                    OPCODE_WRITE). endpoint_id is firmware-build-specific --
                    look it up in the flat_endpoints.json shipped with the
                    ODrive's exact firmware release (e.g.
                    axis0.controller.config.vel_ramp_rate). Only float32
                    endpoints are supported by this command.
"""

import math
import re
import threading
import time
from typing import List, Dict, Any

from ament_index_python import get_package_share_directory
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, Quaternion, TransformStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String, Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import serial.tools.list_ports

_STM_VID = 0x0483
_STM_PID = 0x374E


def _find_stm_port(fallback: str) -> str:
    for p in serial.tools.list_ports.comports():
        if p.vid == _STM_VID and p.pid == _STM_PID:
            return p.device
    return fallback


def _yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    """Convert a yaw angle in radians into a geometry_msgs/Quaternion.

    Used as a fallback for nav_msgs/Odometry.pose.orientation and for
    sensor_msgs/Imu.orientation when the firmware did not supply a quaternion
    on a given telemetry packet. Roll and pitch are assumed to be zero for the
    planar omnidirectional base.
    """
    half = 0.5 * yaw_rad
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

AXIS_STATES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION", 4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIB",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN",
    10: "ENCODER_DIR_FIND", 11: "HOMING",
    12: "HALL_POLARITY_CALIB", 13: "HALL_PHASE_CALIB",
}
CONTROL_MODES = {0: "VOLTAGE", 1: "TORQUE", 2: "VELOCITY", 3: "POSITION"}
INPUT_MODES   = {0: "INACTIVE", 1: "PASSTHROUGH", 2: "VEL_RAMP",
                 3: "POS_FILTER", 5: "TRAP_TRAJ", 6: "TORQUE_RAMP"}

CFG_CLEAR_ERRORS  = 0x20
CFG_SET_STATE     = 0x21
CFG_SET_CTRL_MODE = 0x22
CFG_SET_LIMITS    = 0x23
CFG_SET_POS_GAIN  = 0x24
CFG_SET_VEL_GAINS = 0x25
CFG_STARTUP       = 0x26
CFG_REBOOT        = 0x27
CFG_SET_TORQUE    = 0x28
CFG_STOP          = 0x29
CFG_SET_INPUT_POS = 0x30
CFG_SET_PARAM_FLOAT = 0x31

# ─────────────────────────────────────────────────────────────────────────────
#  TELEMETRY FIELD INDEX MAP
#  Mirrors the comment block above the printf()s in
#  firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Src/main.c:Start_UART_TX_Task.
#
#  The firmware emits each field as `i=value` with `i` the positional index
#  below; the parser translates back to legacy named keys so the body of this
#  function stays identical to the pre-compaction parser.
#
#  Two line shapes share the same numbering:
#    SLIM line (every cycle): emits indices 3, 12, 20..38 — 21 fields, the
#                             EKF output + the two raw-IMU fields that feed
#                             /odrive/imu and /odrive/odom at high rate.
#    FAT  line (every 5th):   emits indices 0..95 — full diagnostic snapshot.
#
#  If the firmware printf is reordered or extended, edit THIS list and the
#  firmware in lock-step, then rebuild the omnidriver package.
# ─────────────────────────────────────────────────────────────────────────────
TELEM_FIELDS = [
    # 0..2  CMD twist
    'CMD_vx', 'CMD_vy', 'CMD_wz',
    # 3..5  IMU Euler (legacy, degrees)
    'IMU_yaw', 'IMU_roll', 'IMU_pitch',
    # 6..9  IMU quaternion
    'IMU_qx', 'IMU_qy', 'IMU_qz', 'IMU_qw',
    # 10..12  IMU angular velocity
    'IMU_wx', 'IMU_wy', 'IMU_wz',
    # 13..15  IMU linear acceleration
    'IMU_ax', 'IMU_ay', 'IMU_az',
    # 16..19  IK wheel speeds
    'IK_u0', 'IK_u1', 'IK_u2', 'IK_u3',
    # 20..23  EKF pose
    'ODOM_phi', 'ODOM_x', 'ODOM_y', 'ODOM_z',
    # 24..27  EKF orientation quaternion
    'ODOM_qx', 'ODOM_qy', 'ODOM_qz', 'ODOM_qw',
    # 28..30  World-frame twist
    'ODOM_w', 'ODOM_vx', 'ODOM_vy',
    # 31..32  Body-frame linear twist
    'ODOM_vxb', 'ODOM_vyb',
    # 33..35  Pose covariance diagonal
    'ODOM_var_x', 'ODOM_var_y', 'ODOM_var_yaw',
    # 36..38  Twist covariance diagonal
    'ODOM_var_vx', 'ODOM_var_vy', 'ODOM_var_wz',
    # 39..51  Axis 0 block (N, E, S, C, P, V, Sh, CPR, Vbus, Ibus, IqSet, IqMeas, U)
    'N0', 'E0', 'S0', 'C0', 'P0', 'V0', 'Sh0', 'CPR0',
    'Vbus0', 'Ibus0', 'IqSet0', 'IqMeas0', 'U0',
    # 52..64  Axis 1 block
    'N1', 'E1', 'S1', 'C1', 'P1', 'V1', 'Sh1', 'CPR1',
    'Vbus1', 'Ibus1', 'IqSet1', 'IqMeas1', 'U1',
    # 65..77  Axis 2 block
    'N2', 'E2', 'S2', 'C2', 'P2', 'V2', 'Sh2', 'CPR2',
    'Vbus2', 'Ibus2', 'IqSet2', 'IqMeas2', 'U2',
    # 78..90  Axis 3 block
    'N3', 'E3', 'S3', 'C3', 'P3', 'V3', 'Sh3', 'CPR3',
    'Vbus3', 'Ibus3', 'IqSet3', 'IqMeas3', 'U3',
    # 91..94  BT state
    'BT_active', 'BT_vx', 'BT_vy', 'BT_wz',
    # 95  ESP32 age, ms (-1 sentinel = no Type-3 message yet)
    'ESP32_age_ms',
    # 96  Firmware SM state — distinguishes SM_IDLE (commanded) from
    #     SM_ESTOP (hardware button held) which both leave axis_state=IDLE
    #     0=BOOT, 1=STARTUP, 2=RUNNING, 3=IDLE, 4=ESTOP
    'SM_state',
]

# Pretty labels for telemetry pills / logs. Must match firmware ODriveSMState.
SM_STATE_LABELS = {
    0: 'BOOT',
    1: 'STARTUP',
    2: 'RUNNING',
    3: 'IDLE',
    4: 'ESTOP',
}

# Firmware error code descriptions — must match FERR_* defines in main.h
FERR_DESCRIPTIONS = {
    0x01: 'CAN TX timeout',
    0x02: 'CAN bus-off recovered',
    0x03: 'CAN RX read fail',
    0x10: 'ODrive startup failed',
    0x20: 'Arm timeout (all retries failed)',
    0x21: 'Axis fell out of CLOSED_LOOP — auto-rearm issued',
    0x30: 'ODrive axis fault (AXIS_Error non-zero)',
    0x31: 'Command watchdog fired — no SET_VEL received',
    0x32: 'Heartbeat timeout — axis stopped responding',
    0x40: 'IMU sh2_open failed',
    0x41: 'IMU setSensorCallback failed',
    0x42: 'IMU setSensorConfig failed',
    0x43: 'IMU reset detected — re-configuring',
    0x50: 'BT queue full',
    0x51: 'BT parse fail',
    0x60: 'Stack watermark low',
    0x61: 'Stack overflow detected',
}

# ─────────────────────────────────────────────────────────────────────────────
#  DASHBOARD HTML
# ─────────────────────────────────────────────────────────────────────────────
from pathlib import Path
DASHBOARD_HTML = (
    Path(get_package_share_directory('omnidriver')) / 'assets' / 'dashboard.html'
).read_text()


# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────
class ODriveDashboardNode(Node):

    def __init__(self):
        super().__init__('odrive_dashboard_node')

        self.declare_parameter('serial_port',         '/dev/ttyOmniSTM32')
        self.declare_parameter('baud_rate',           230400)
        # nav2 on Humble (and teleop_twist_keyboard) publishes UNSTAMPED
        # geometry_msgs/Twist on /cmd_vel, so default to that. Set true only if
        # your velocity source publishes geometry_msgs/TwistStamped (newer nav2
        # with enable_stamped_cmd_vel:=true, or teleop --stamped). Both callbacks
        # forward linear.x/linear.y/angular.z, so holonomic strafing works either way.
        self.declare_parameter('use_stamped_cmd_vel', False)
        self.declare_parameter('tx_period',           0.1)
        # Wheel order must match StartODriveTask odrives[].NODE_ID in the firmware.
        self.declare_parameter('node_ids',            [36, 34, 33, 40])
        self.declare_parameter('enable_web_gui',      False)
        self.declare_parameter('web_gui_port',        5000)
        # Frame ids used in the ROS Imu and Odometry messages.
        self.declare_parameter('odom_frame_id',       'odom')
        self.declare_parameter('base_frame_id',       'base_link')
        # Publish the IMU directly in base_link. The BNO085 is body-aligned and
        # the firmware already corrects its yaw to the base_link sense, so this
        # is valid AND it means robot_localization needs NO imu_link->base_link
        # TF lookup to fuse it. (A flaky/unresolved lookup silently drops the
        # IMU while odom -- whose twist is already in base_link -- keeps
        # fusing, which is exactly the "yaw covariance explodes, orientation
        # stuck at identity" symptom seen before.)
        self.declare_parameter('imu_frame_id',        'base_link')
        # When True, the dashboard broadcasts the odom -> base_link TF -- this
        # replaces robot_localization's `publish_tf: True` in
        # omni_basics.launch.py now that the on-MCU EKF supplies a filtered
        # pose+covariance directly. Set to False if running an external EKF
        # (e.g. robot_localization) alongside, to avoid two publishers
        # fighting over the same transform. See the home-custom-base repo's
        # omnibase_documentation.md section 12 for the staged-rollout order
        # (flash firmware -> update this node -> only then drop
        # robot_localization from the launch file).
        self.declare_parameter('publish_tf',          True)
        # Live linear-odometry scale factors for distance calibration (1.0 =
        # unchanged). Forward and lateral slip differently on a mecanum base, so
        # they are independent. Tune WITHOUT reflashing/rebuilding, e.g.:
        #   ros2 param set /odrive_dashboard_node linear_scale_x 1.08
        # Procedure: drive a tape-measured distance, set scale = real/measured.
        # x = 0.97: from a 2 m forward calibration (2026-06-01). y not yet
        # calibrated (strafe run pending) so left at 1.0. Applied to the EKF's
        # body-frame twist (ODOM_vxb/vyb) -- these are empirical wheel-radius
        # corrections, independent of the EKF rollout, and still apply.
        self.declare_parameter('linear_scale_x', 0.97)
        self.declare_parameter('linear_scale_y', 1.0)

        port             = _find_stm_port(self.get_parameter('serial_port').value)
        baud             = self.get_parameter('baud_rate').value
        self.use_stamped = self.get_parameter('use_stamped_cmd_vel').value
        self.tx_period   = self.get_parameter('tx_period').value
        self.expected_node_ids: List[int] = list(
            self.get_parameter('node_ids').value)
        enable_web = self.get_parameter('enable_web_gui').value
        web_port   = self.get_parameter('web_gui_port').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.imu_frame_id  = self.get_parameter('imu_frame_id').value
        self.publish_tf    = self.get_parameter('publish_tf').value
        # TF broadcaster for odom -> base_link. Created unconditionally because
        # `publish_tf` is a parameter that can be flipped at runtime via the
        # on-set-parameters callback below.
        self._tf_broadcaster = TransformBroadcaster(self)

        self.add_on_set_parameters_callback(self._on_params_changed)

        self.declare_parameter('demo_mode', False)
        self._demo = self.get_parameter('demo_mode').value
        # Follow mode: add the reactive "unload-the-arm" base yaw published on
        # /follow/base_yaw into the commanded wz (only while the value is fresh).
        self.declare_parameter('follow_base_yaw_enabled', False)
        self._baud = baud
        self._ser_lock = threading.Lock()
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Serial: {port} @ {baud}")
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial unavailable ({e}) — running in demo mode")
            self.ser = None
            self._demo = True
        self.get_logger().info(f"Serial: {port} @ {baud}")

        self.tx_vx: float = 0.0
        self.tx_vy: float = 0.0
        self.tx_wz: float = 0.0
        self.tx_count: int = 0
        # Reactive base-yaw injected by the arm follow controller
        # (/follow/base_yaw). Added to wz only while FRESH (<0.5 s old).
        self._follow_base_yaw: float = 0.0
        self._follow_base_yaw_time: float = 0.0
        self.discovered_node_ids: List[int] = []
        self._latest_telem: Dict[str, Any] = self._empty_telem()
        self._sio = None

        # ── connection-link monitoring ───────────────────────────────────
        # Monotonic-time (seconds) when the most recent /odrive/raw line was
        # received from the STM32. None means "no line yet". Read & written
        # only from this process; the dashboard's STM32→PC age is computed
        # from this value.
        self._last_raw_msg_time: float | None = None
        # Latest ESP32→STM32 age (ms) as reported by the firmware. -1 sentinel
        # (or None) means "the STM32 has never received an ESP32 message".
        self._latest_esp32_age_ms: int | None = None
        # Last accepted IMU yaw (deg); used to reject corrupted/outlier telemetry
        # lines before they reach ROS. The on-MCU EKF now does its own outlier
        # rejection (firmware-side), so this is defence-in-depth, not load-
        # bearing -- kept per the EKF rollout's "keep unchanged" guidance.
        self._last_imu_yaw: float | None = None

        # publishers
        self.pub_raw         = self.create_publisher(String,            'odrive/raw',               10)
        self.pub_debug       = self.create_publisher(String,            'odrive/debug',             10)
        self.pub_cmd         = self.create_publisher(Float32MultiArray, 'odrive/cmd_twist',         10)
        # Legacy Float32MultiArray topic kept for the web dashboard.
        self.pub_imu_euler   = self.create_publisher(Float32MultiArray, 'odrive/imu_euler',         10)
        # Standard ROS 2 IMU message: orientation quaternion, angular velocity,
        # and linear acceleration sourced from the BNO085 telemetry fields.
        self.pub_imu         = self.create_publisher(Imu,               'odrive/imu',               10)
        self.pub_ik          = self.create_publisher(Float32MultiArray, 'odrive/ik_wheel_speeds',   10)
        # Standard ROS 2 Odometry message built from STM32 ODOM_* fields.
        self.pub_odom        = self.create_publisher(Odometry,          'odrive/odom',              10)
        self.pub_body_twist  = self.create_publisher(Float32MultiArray, 'odrive/body_twist',        10)
        self.pub_node_ids    = self.create_publisher(Int32MultiArray,   'odrive/node_ids',          10)
        self.pub_axis_errors = self.create_publisher(Int32MultiArray,   'odrive/axis_errors',       10)
        self.pub_axis_states = self.create_publisher(Int32MultiArray,   'odrive/axis_states',       10)
        self.pub_ctrl_status = self.create_publisher(Int32MultiArray,   'odrive/controller_status', 10)
        self.pub_updated     = self.create_publisher(Int32MultiArray,   'odrive/updated',           10)
        self.pub_pos         = self.create_publisher(Float32MultiArray, 'odrive/pos_est',           10)
        self.pub_vel         = self.create_publisher(Float32MultiArray, 'odrive/vel_est',           10)
        self.pub_shadow      = self.create_publisher(Int32MultiArray,   'odrive/encoder_shadow',    10)
        self.pub_cpr         = self.create_publisher(Int32MultiArray,   'odrive/encoder_cpr',       10)
        self.pub_vbus        = self.create_publisher(Float32MultiArray, 'odrive/bus_voltage',       10)
        self.pub_ibus        = self.create_publisher(Float32MultiArray, 'odrive/bus_current',       10)
        self.pub_iq_set      = self.create_publisher(Float32MultiArray, 'odrive/iq_setpoint',       10)
        self.pub_iq_meas     = self.create_publisher(Float32MultiArray, 'odrive/iq_measured',       10)
        self.pub_discovered  = self.create_publisher(Int32MultiArray,   'odrive/discovered_nodes',  10)
        self.pub_states_str  = self.create_publisher(String,            'odrive/axis_states_str',   10)

        # subscribers
        if self.use_stamped:
            self.create_subscription(TwistStamped, 'cmd_vel',
                                     self._cb_cmd_vel_stamped, 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self._cb_cmd_vel, 10)

        # Reactive base-yaw from the arm follow controller (unload-the-arm).
        self.create_subscription(Float64, '/follow/base_yaw',
                                 self._cb_follow_base_yaw, 10)

        self.create_subscription(String, 'odrive/config_cmd',
                                 self._cb_config_cmd, 10)

        self.create_timer(self.tx_period, self._send_control_cmd)
        # Periodic refresh for the connection-status widget: recomputes the
        # ESP32→STM32 and STM32→PC ages from the cached timestamps and pushes
        # them to the web dashboard. Runs even when no UART data is arriving,
        # so the "STM32→PC" reading grows visibly during a disconnect.
        self.create_timer(0.25, self._tick_link_status)
        threading.Thread(target=self._receiver, daemon=True).start()

        if enable_web:
            self._start_web_server(web_port)

        self.get_logger().info("ODrive Dashboard Node ready.")

    # ── web server ────────────────────────────────────────────────────────────

    def _start_web_server(self, port: int):
        try:
            from flask import Flask, render_template_string, send_from_directory
            from flask_socketio import SocketIO
        except ImportError:
            self.get_logger().error(
                "enable_web_gui requires:  pip install flask flask-socketio")
            return

        app = Flask(__name__)
        app.config['SECRET_KEY'] = 'odrive-dashboard-secret'
        sio = SocketIO(app, cors_allowed_origins='*',
                       async_mode='threading', logger=False,
                       engineio_logger=False)
        self._sio = sio

        @app.route('/')
        def index():
            return render_template_string(DASHBOARD_HTML)

        @app.route('/js/<path:filename>')
        def serve_js(filename):
            # Try installed share dir first, fall back to source tree next to this file
            for candidate in [
                Path(get_package_share_directory('omnidriver')) / 'assets' / 'js',
                Path(__file__).parent.parent / 'assets' / 'js',
            ]:
                if candidate.exists():
                    return send_from_directory(str(candidate), filename)
            return 'JS file not found — see status.txt for setup instructions', 404

        @sio.on('connect')
        def on_connect():
            self.get_logger().info("Web client connected")
            sio.emit('telemetry', self._latest_telem)

        @sio.on('command')
        def on_command(data):
            self._handle_web_command(data)

        @sio.on('disconnect')
        def on_disconnect():
            # SAFETY: the web UI latches a velocity and _send_control_cmd streams
            # it at 10 Hz forever. If the operator closes the tab / loses Wi-Fi
            # while a non-zero velocity is set, that stale velocity keeps being
            # commanded (and the firmware command-watchdog never fires because
            # packets keep arriving). Zero the streamed velocity on disconnect so
            # the base stops when the operator loses the UI.
            self.tx_vx = self.tx_vy = self.tx_wz = 0.0
            self.get_logger().warn("Web client disconnected -- zeroing commanded velocity")

        def _run():
            # allow_unsafe_werkzeug: this is a local robot dashboard, not a
            # public production server. Newer flask_socketio raises unless we
            # explicitly opt in to Werkzeug's dev server.
            sio.run(app, host='0.0.0.0', port=port,
                    use_reloader=False, log_output=False,
                    allow_unsafe_werkzeug=True)

        threading.Thread(target=_run, daemon=True).start()
        self.get_logger().info(
            f"Web dashboard at  http://localhost:{port}")

    def _handle_web_command(self, data: dict):
        cmd  = data.get('type', '')
        mask = int(data.get('mask', 0x0F)) & 0x0F
        mhex = f'{mask:#04x}'

        if cmd == 'ctrl':
            vx = float(data.get('vx', 0.0))
            vy = float(data.get('vy', 0.0))
            wz = float(data.get('wz', 0.0))
            self.tx_vx, self.tx_vy, self.tx_wz = vx, vy, wz
            self._serial_write(f"1 {vx:.4f} {vy:.4f} {wz:.4f}")

        elif cmd == 'startup':
            # Zero the latched stream so re-arming never replays a stale velocity
            # (which would make the base lurch the instant it re-enters closed loop).
            self.tx_vx = self.tx_vy = self.tx_wz = 0.0
            cm = int(data.get('ctrl_mode', 2))
            im = int(data.get('input_mode', 1))
            st = int(data.get('state', 8))
            self._serial_write(f"2 {CFG_STARTUP} {mhex} {cm} {im} {st}")

        elif cmd == 'stop':
            # Stop must also zero the latched velocity, otherwise _send_control_cmd
            # keeps streaming the old non-zero command and the base resumes the
            # moment it is re-armed.
            self.tx_vx = self.tx_vy = self.tx_wz = 0.0
            self._serial_write(f"2 {CFG_STOP} {mhex}")

        elif cmd == 'clear_errors':
            self._serial_write(f"2 {CFG_CLEAR_ERRORS} {mhex}")

        elif cmd == 'reboot':
            self.tx_vx = self.tx_vy = self.tx_wz = 0.0
            self._serial_write(f"2 {CFG_REBOOT} {mhex}")

        elif cmd == 'set_state':
            self.tx_vx = self.tx_vy = self.tx_wz = 0.0
            self._serial_write(
                f"2 {CFG_SET_STATE} {mhex} {int(data.get('state',1))}")

        elif cmd == 'set_ctrl_mode':
            cm = int(data.get('ctrl_mode', 2))
            im = int(data.get('input_mode', 1))
            self._serial_write(f"2 {CFG_SET_CTRL_MODE} {mhex} {cm} {im}")

        elif cmd == 'set_limits':
            vl = float(data.get('vel_lim', 10.0))
            cl = float(data.get('curr_lim', 20.0))
            self._serial_write(
                f"2 {CFG_SET_LIMITS} {mhex} {vl:.4f} {cl:.4f}")

        elif cmd == 'set_pos_gain':
            self._serial_write(
                f"2 {CFG_SET_POS_GAIN} {mhex} {float(data.get('pos_gain',20.0)):.4f}")

        elif cmd == 'set_vel_gains':
            vg  = float(data.get('vel_gain',     0.16))
            vig = float(data.get('vel_int_gain', 0.32))
            self._serial_write(
                f"2 {CFG_SET_VEL_GAINS} {mhex} {vg:.4f} {vig:.4f}")

        elif cmd == 'set_torque':
            self._serial_write(
                f"2 {CFG_SET_TORQUE} {mhex} {float(data.get('torque',0.0)):.4f}")

        elif cmd == 'set_input_pos':
            pos = float(data.get('pos', 0.0))
            vff = float(data.get('vel_ff', 0.0))
            tff = float(data.get('torque_ff', 0.0))
            self._serial_write(
                f"2 {CFG_SET_INPUT_POS} {mhex} {pos:.4f} {vff:.4f} {tff:.4f}")

        elif cmd == 'set_param_float':
            # Arbitrary ODrive parameter write over CAN (RxSdo). endpoint_id
            # is firmware-build-specific -- look it up in the flat_endpoints.json
            # for the ODrive's exact firmware release.
            endpoint_id = int(data.get('endpoint_id', 0))
            value = float(data.get('value', 0.0))
            self._serial_write(
                f"2 {CFG_SET_PARAM_FLOAT} {mhex} {endpoint_id} {value:.6f}")

        else:
            self.get_logger().warn(f"Unknown web command: {cmd}")

        if self._sio:
            self._sio.emit('log', {'text': f'{cmd} mask={mhex}', 'level': 'i'})

    # ── parameter callback ────────────────────────────────────────────────────

    def _on_params_changed(self, params):
        for p in params:
            if p.name == 'tx_period':
                self.tx_period = p.value
            elif p.name == 'publish_tf':
                self.publish_tf = p.value
        return SetParametersResult(successful=True)

    # ── subscribers ───────────────────────────────────────────────────────────

    def _cb_cmd_vel(self, msg: Twist):
        self.tx_vx, self.tx_vy, self.tx_wz = \
            msg.linear.x, msg.linear.y, msg.angular.z

    def _cb_cmd_vel_stamped(self, msg: TwistStamped):
        self.tx_vx, self.tx_vy, self.tx_wz = \
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z

    def _cb_follow_base_yaw(self, msg: Float64):
        # Reactive yaw from the arm follow controller; merged into wz at TX time.
        self._follow_base_yaw = float(msg.data)
        self._follow_base_yaw_time = time.monotonic()

    def _cb_config_cmd(self, msg: String):
        line = msg.data.strip()
        if not line.startswith('2 '):
            self.get_logger().warn(f"config_cmd ignored (must start '2 '): {line}")
            return
        self._serial_write(line)

    # ── periodic TX ───────────────────────────────────────────────────────────

    def _send_control_cmd(self):
        wz = self.tx_wz
        # Follow mode: add the reactive "unload-the-arm" base yaw if enabled and
        # FRESH (<0.5 s). Stale or disabled -> normal driving is unchanged.
        if self.get_parameter('follow_base_yaw_enabled').value and \
                (time.monotonic() - self._follow_base_yaw_time) < 0.5:
            wz += self._follow_base_yaw
        self._serial_write(
            f"1 {self.tx_vx:.4f} {self.tx_vy:.4f} {wz:.4f}")
        self.tx_count += 1

    # ── public helpers ────────────────────────────────────────────────────────

    def send_startup(self, cm=2, im=1, state=8, mask=0x0F):
        self._serial_write(f"2 {CFG_STARTUP} {mask:#04x} {cm} {im} {state}")

    def send_stop(self, mask=0x0F):
        self._serial_write(f"2 {CFG_STOP} {mask:#04x}")

    def send_clear_errors(self, mask=0x0F):
        self._serial_write(f"2 {CFG_CLEAR_ERRORS} {mask:#04x}")

    def send_reboot(self, mask=0x0F):
        self._serial_write(f"2 {CFG_REBOOT} {mask:#04x}")

    def send_set_limits(self, vl, cl, mask=0x0F):
        self._serial_write(f"2 {CFG_SET_LIMITS} {mask:#04x} {vl:.4f} {cl:.4f}")

    def send_set_pos_gain(self, pg, mask=0x0F):
        self._serial_write(f"2 {CFG_SET_POS_GAIN} {mask:#04x} {pg:.4f}")

    def send_set_vel_gains(self, vg, vi, mask=0x0F):
        self._serial_write(f"2 {CFG_SET_VEL_GAINS} {mask:#04x} {vg:.4f} {vi:.4f}")

    def send_set_param_float(self, endpoint_id, value, mask=0x0F):
        """Write an arbitrary ODrive float32 parameter over CAN (RxSdo).
        endpoint_id comes from the flat_endpoints.json matching the ODrive's
        exact firmware release (e.g. axis0.controller.config.vel_ramp_rate)."""
        self._serial_write(
            f"2 {CFG_SET_PARAM_FLOAT} {mask:#04x} {int(endpoint_id)} {value:.6f}")

    def send_set_controller_mode(self, cm, im, mask=0x0F):
        self._serial_write(f"2 {CFG_SET_CTRL_MODE} {mask:#04x} {cm} {im}")

    # ── serial ────────────────────────────────────────────────────────────────

    def _serial_write(self, text: str):
        with self._ser_lock:
            if self.ser is None:
                return
            try:
                self.ser.write((text + '\r\n').encode())
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    # ── RX thread ─────────────────────────────────────────────────────────────

    def _receiver(self):
        reconnect_delay = 2.0
        while rclpy.ok():
            with self._ser_lock:
                ser = self.ser

            if ser is None:
                if self._demo:
                    return
                port = _find_stm_port(self.get_parameter('serial_port').value)
                self.get_logger().info(f"Reconnecting serial on {port}...")
                try:
                    new_ser = serial.Serial(port, self._baud, timeout=0.1)
                    with self._ser_lock:
                        self.ser = new_ser
                    self.discovered_node_ids.clear()
                    self.get_logger().info(f"Serial reconnected: {port}")
                except serial.SerialException as e:
                    self.get_logger().warn(
                        f"Reconnect failed ({e}), retrying in {reconnect_delay:.0f}s")
                    time.sleep(reconnect_delay)
                continue

            try:
                line = ser.readline().decode(errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read: {e}")
                with self._ser_lock:
                    if self.ser is ser:
                        try:
                            self.ser.close()
                        except Exception:
                            pass
                        self.ser = None
                time.sleep(0.5)
                continue
            if not line:
                continue
            # Stamp the STM32→PC link timestamp on every line we successfully
            # read off the serial port, independently of whether the parser
            # later finds usable fields. A malformed line still proves the
            # link is alive.
            self._last_raw_msg_time = time.monotonic()
            self.pub_raw.publish(String(data=line))
            self._parse_and_publish(line)

    # ── link-status helpers ──────────────────────────────────────────────────

    # Thresholds (seconds). The default ROS<->STM32 telemetry rate is ~100 Hz
    # (firmware Start_UART_TX_Task: osDelay(10)); the ESP32 heartbeat runs
    # well above 2 Hz. 0.5 s already represents many missed packets at either
    # rate, so OK / WARN / LOST is a sensible split.
    _LINK_OK_S   = 0.5
    _LINK_WARN_S = 2.0

    @classmethod
    def _link_status_label(cls, age_s):
        if age_s is None:
            return 'UNKNOWN'
        if age_s < cls._LINK_OK_S:
            return 'OK'
        if age_s < cls._LINK_WARN_S:
            return 'WARN'
        return 'LOST'

    def _inject_link_ages(self, telem: dict) -> None:
        """Populate ESP32->STM32 and STM32->PC link ages on the telemetry dict.

        The dict gets four fields:
          esp32_age_ms / esp32_status : ESP32→STM32 link, from the firmware.
          pc_age_ms    / pc_status    : STM32→PC link, measured here.

        Ages are integer milliseconds, or `None` when no message has ever
        been observed (rendered as "unknown" on the dashboard).
        """
        # ESP32→STM32 — the firmware sends -1 when no ESP32 message has been
        # received yet; _parse_and_publish() converts that to None.
        esp32_ms = self._latest_esp32_age_ms
        # PC age — time.monotonic is robust across system-clock jumps.
        if self._last_raw_msg_time is None:
            pc_ms = None
        else:
            pc_ms = int((time.monotonic() - self._last_raw_msg_time) * 1000)
        telem['esp32_age_ms'] = esp32_ms
        telem['pc_age_ms']    = pc_ms
        telem['esp32_status'] = self._link_status_label(
            None if esp32_ms is None else esp32_ms / 1000.0)
        telem['pc_status']    = self._link_status_label(
            None if pc_ms is None else pc_ms / 1000.0)

    def _tick_link_status(self) -> None:
        """ROS timer callback: refresh link ages in the cached telemetry and
        push to the web dashboard so the displayed values keep advancing even
        when the STM32 stops sending data."""
        self._inject_link_ages(self._latest_telem)
        if self._sio:
            self._sio.emit('telemetry', self._latest_telem)

    def _parse_and_publish(self, line: str):
        try:
            # Firmware error event:  E=<code>,<axis>,<detail>
            if line.startswith('E='):
                parts = line[2:].split(',')
                if len(parts) == 3:
                    code, axis, detail = int(parts[0]), int(parts[1]), int(parts[2])
                    desc = FERR_DESCRIPTIONS.get(code, f'Unknown error 0x{code:02X}')
                    axis_s = 'sys' if axis == 255 else f'axis {axis}'
                    msg = f'[FW ERR] {desc} ({axis_s}, detail={detail})'
                    self.get_logger().warn(msg)
                    if self._sio:
                        self._sio.emit('firmware_error', {
                            'code': code, 'axis': axis, 'detail': detail,
                            'desc': desc, 'text': msg,
                        })
                return

            # Lost-error counter:  ELOST=<n>
            if line.startswith('ELOST='):
                n = int(line[6:])
                msg = f'[FW ERR] {n} firmware error(s) were lost (queue overflow)'
                self.get_logger().error(msg)
                if self._sio:
                    self._sio.emit('firmware_error', {
                        'code': 0xFF, 'axis': 255, 'detail': n,
                        'desc': 'Errors lost', 'text': msg,
                    })
                return

            # Firmware emits the compact form `i=value` (see TELEM_FIELDS).
            # \w+ accepts the leading digit prefix too; we translate the
            # numeric keys back to legacy named keys so the body of this
            # function stays identical to the pre-compaction parser.
            matches = re.findall(
                r'(\w+)=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', line)
            if not matches:
                return
            data: dict = {}
            for k, v in matches:
                if k.isdigit():
                    idx = int(k)
                    if 0 <= idx < len(TELEM_FIELDS):
                        data[TELEM_FIELDS[idx]] = float(v)
                else:
                    # Forward-compat: accept either compact (digit) or named
                    # keys, so a half-flashed setup (old firmware + new node,
                    # or vice-versa) still produces a usable telemetry frame.
                    data[k] = float(v)
            if not data:
                return

            f, i = self._sf, self._si

            # Reject corrupted/garbled telemetry BEFORE it reaches ROS. The
            # UART line has no checksum, so occasionally a line arrives
            # garbled and IMU_yaw parses to a wrong-but-numeric value. The
            # on-MCU EKF now does its own outlier rejection (see the
            # home-custom-base omnibase_documentation.md section 11.3), so
            # this is defence-in-depth, not load-bearing -- kept rather than
            # removed per the EKF rollout's "keep unchanged" guidance.
            if 'IMU_yaw' in data:
                iy = data['IMU_yaw']
                if not math.isfinite(iy) or abs(iy) > 200.0:
                    return
                if self._last_imu_yaw is not None:
                    dyaw = (iy - self._last_imu_yaw + 180.0) % 360.0 - 180.0
                    if abs(dyaw) > 40.0:
                        return
                self._last_imu_yaw = iy

            # FAST PATH: the slim high-rate line (sent every cycle, fields
            # 3, 12, 20..38, 96 — see the index map above main.c's printf)
            # carries no per-axis block (no 'N0='). Falling through to the
            # full path below would default every axis/Vbus/etc. field to 0
            # and overwrite the good values from the last fat line, making
            # the dashboard flicker between real data and zeros/UNDEFINED.
            # Just refresh the IMU/EKF-odom fields on the cached telemetry
            # and emit that instead.
            if 'N0' not in data:
                telem = dict(self._latest_telem)
                if 'IMU_yaw' in data:
                    telem['imu_yaw'] = f(data.get('IMU_yaw'))
                if 'IMU_wz' in data:
                    telem['imu_wz'] = f(data.get('IMU_wz'))
                if 'ODOM_phi' in data:
                    telem['odom_phi'] = f(data.get('ODOM_phi'))
                    telem['odom_x']   = f(data.get('ODOM_x'))
                    telem['odom_y']   = f(data.get('ODOM_y'))
                    telem['odom_w']   = f(data.get('ODOM_w'))
                    telem['odom_vx']  = f(data.get('ODOM_vx'))
                    telem['odom_vy']  = f(data.get('ODOM_vy'))
                if 'SM_state' in data:
                    sm_state = int(data['SM_state'])
                    telem['sm_state']       = sm_state
                    telem['sm_state_label'] = SM_STATE_LABELS.get(sm_state, 'UNKNOWN')
                self._inject_link_ages(telem)
                self._latest_telem = telem
                if self._sio:
                    self._sio.emit('telemetry', telem)
                return

            node_ids    = [i(data.get(f'N{j}'))        for j in range(4)]
            axis_errors = [i(data.get(f'E{j}'))        for j in range(4)]
            axis_states = [i(data.get(f'S{j}'))        for j in range(4)]
            ctrl_status = [i(data.get(f'C{j}'))        for j in range(4)]
            updated     = [i(data.get(f'U{j}'))        for j in range(4)]
            pos_est     = [f(data.get(f'P{j}'))        for j in range(4)]
            vel_est     = [f(data.get(f'V{j}'))        for j in range(4)]
            shadows     = [i(data.get(f'Sh{j}'))       for j in range(4)]
            cprs        = [i(data.get(f'CPR{j}'))      for j in range(4)]
            vbus        = [f(data.get(f'Vbus{j}'))     for j in range(4)]
            ibus        = [f(data.get(f'Ibus{j}'))     for j in range(4)]
            iq_set      = [f(data.get(f'IqSet{j}'))    for j in range(4)]
            iq_meas     = [f(data.get(f'IqMeas{j}'))   for j in range(4)]

            cmd_vx = f(data.get('CMD_vx')); cmd_vy = f(data.get('CMD_vy'))
            cmd_wz = f(data.get('CMD_wz'))

            # Legacy Euler telemetry (degrees, BNO085 yaw/pitch/roll).
            imu_y  = f(data.get('IMU_yaw')); imu_r = f(data.get('IMU_roll'))
            imu_p  = f(data.get('IMU_pitch'))

            # New IMU telemetry fields from the STM32 (sensor_msgs/Imu compatible):
            #   orientation quaternion  (unitless, (x, y, z, w))
            #   angular velocity        (rad/s, body frame)
            #   linear acceleration     (m/s^2, body frame, gravity removed)
            # data.get() returns None when a field is missing — `f()` then
            # falls back to 0.0 so a truncated telemetry line cannot crash us.
            imu_qx = f(data.get('IMU_qx')); imu_qy = f(data.get('IMU_qy'))
            imu_qz = f(data.get('IMU_qz')); imu_qw = f(data.get('IMU_qw'))
            imu_wx = f(data.get('IMU_wx')); imu_wy = f(data.get('IMU_wy'))
            imu_wz = f(data.get('IMU_wz'))
            imu_ax = f(data.get('IMU_ax')); imu_ay = f(data.get('IMU_ay'))
            imu_az = f(data.get('IMU_az'))
            # True once we have seen at least one quaternion field in this packet.
            imu_quat_present = any(k in data for k in
                                   ('IMU_qx', 'IMU_qy', 'IMU_qz', 'IMU_qw'))

            ik     = [f(data.get(f'IK_u{j}')) for j in range(4)]
            o_phi  = f(data.get('ODOM_phi')); o_x = f(data.get('ODOM_x'))
            o_y    = f(data.get('ODOM_y'));   o_w = f(data.get('ODOM_w'))
            o_vx   = f(data.get('ODOM_vx')); o_vy = f(data.get('ODOM_vy'))
            # EKF-extended fields emitted by the firmware after the on-MCU
            # EKF landed. All are optional — older firmware just leaves them
            # as 0.0.
            o_z    = f(data.get('ODOM_z'))
            o_qx   = f(data.get('ODOM_qx')); o_qy = f(data.get('ODOM_qy'))
            o_qz   = f(data.get('ODOM_qz')); o_qw = f(data.get('ODOM_qw'))
            o_vxb  = f(data.get('ODOM_vxb')); o_vyb = f(data.get('ODOM_vyb'))
            # Covariance diagonal (only the values the firmware actually
            # estimates — z, roll, pitch, wx, wy are advertised as 1e9
            # below so downstream filters know they're unknown).
            o_var_x   = f(data.get('ODOM_var_x'))
            o_var_y   = f(data.get('ODOM_var_y'))
            o_var_yaw = f(data.get('ODOM_var_yaw'))
            o_var_vx  = f(data.get('ODOM_var_vx'))
            o_var_vy  = f(data.get('ODOM_var_vy'))
            o_var_wz  = f(data.get('ODOM_var_wz'))
            # Detect "EKF firmware present" once per packet — if all the
            # ODOM_q*/ODOM_var_* fields are missing we fall back to the
            # legacy yaw-derived quaternion and the old placeholder
            # covariance so the topic shape stays valid for any consumer.
            ekf_quat_present = any(k in data for k in
                                   ('ODOM_qx', 'ODOM_qy', 'ODOM_qz', 'ODOM_qw'))
            ekf_cov_present  = any(k in data for k in
                                   ('ODOM_var_x', 'ODOM_var_y', 'ODOM_var_yaw',
                                    'ODOM_var_vx', 'ODOM_var_vy', 'ODOM_var_wz'))
            bt_active = i(data.get('BT_active'))

            # ESP32→STM32 link age, reported in milliseconds by the firmware.
            # The STM32 sends -1 when no ESP32 message has ever arrived; we
            # carry that through as `None` for the dashboard so it can show
            # "unknown" instead of a misleading number. A missing field
            # (older firmware) is also treated as unknown.
            if 'ESP32_age_ms' in data:
                esp32_raw = i(data.get('ESP32_age_ms'))
                self._latest_esp32_age_ms = (
                    None if esp32_raw < 0 else esp32_raw)
            # else: keep the previous cached value (no overwrite).

            def _f32(l): m=Float32MultiArray(); m.data=l; return m
            def _i32(l): m=Int32MultiArray();   m.data=l; return m

            self.pub_cmd        .publish(_f32([cmd_vx, cmd_vy, cmd_wz]))
            # Backward-compatible Euler topic (degrees) for the web dashboard.
            self.pub_imu_euler  .publish(_f32([imu_y, imu_r, imu_p]))
            self.pub_ik         .publish(_f32(ik))
            self.pub_body_twist .publish(_f32([o_w, o_vx, o_vy]))

            # ── sensor_msgs/Imu ────────────────────────────────────────────
            # Fill from the IMU_q* / IMU_w* / IMU_a* telemetry fields. If the
            # firmware has not (yet) sent a quaternion this cycle, fall back to
            # an orientation derived from ODOM_phi so downstream consumers
            # always receive a valid unit quaternion.
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame_id
            if imu_quat_present and (imu_qx or imu_qy or imu_qz or imu_qw):
                imu_msg.orientation = Quaternion(
                    x=imu_qx, y=imu_qy, z=imu_qz, w=imu_qw)
            else:
                imu_msg.orientation = _yaw_to_quaternion(o_phi)
            imu_msg.angular_velocity.x = imu_wx
            imu_msg.angular_velocity.y = imu_wy
            imu_msg.angular_velocity.z = imu_wz
            imu_msg.linear_acceleration.x = imu_ax
            imu_msg.linear_acceleration.y = imu_ay
            imu_msg.linear_acceleration.z = imu_az
            # Covariances are not yet calibrated. Per sensor_msgs/Imu, setting
            # element 0 of a covariance matrix to -1 marks the field as
            # unknown; until the BNO085 reports accuracies we publish small
            # placeholder variances on the diagonal so consumers that require
            # a positive-definite covariance (e.g. robot_localization) still
            # work. Flip the lines below to advertise "unknown" instead.
            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ]
            imu_msg.angular_velocity_covariance = [
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.001,
            ]
            imu_msg.linear_acceleration_covariance = [
                0.05, 0.0, 0.0,
                0.0, 0.05, 0.0,
                0.0, 0.0, 0.05,
            ]
            self.pub_imu.publish(imu_msg)

            # ── nav_msgs/Odometry ───────────────────────────────────────────
            # ODOM_x / ODOM_y / ODOM_z in meters (z = 0 on a planar base),
            # ODOM_phi in radians (heading), and:
            #   - ODOM_q*    EKF orientation quaternion (preferred); falls
            #                back to a yaw-derived quaternion when the EKF
            #                firmware hasn't been flashed yet.
            #   - ODOM_vxb / ODOM_vyb : body-frame linear twist — what
            #     nav_msgs/Odometry expects under `child_frame_id`. When the
            #     EKF firmware is absent, ODOM_vx / ODOM_vy carry the
            #     legacy world-frame twist and we fall back to those.
            #   - ODOM_var_* : EKF covariance diagonal mapped into the ROS
            #     [x, y, z, roll, pitch, yaw] / [vx, vy, vz, wx, wy, wz]
            #     layout. Unestimated entries (z / roll / pitch / wx / wy)
            #     are marked with 1e9 so downstream filters treat them as
            #     "unknown" rather than "tightly zero".
            cphi = math.cos(o_phi); sphi = math.sin(o_phi)
            if ekf_quat_present or ekf_cov_present or ('ODOM_vxb' in data):
                vx_body, vy_body = o_vxb, o_vyb
            else:
                # Legacy (pre-EKF) firmware: ODOM_vx/vy are world-frame --
                # undo the firmware's rotation to recover body-frame velocity.
                vx_body =  cphi * o_vx + sphi * o_vy
                vy_body = -sphi * o_vx + cphi * o_vy
            # Live distance calibration (read fresh so `ros2 param set` applies
            # immediately, no restart). 1.0 = unchanged. Empirical wheel-radius
            # corrections -- independent of the EKF rollout, still apply.
            vx_body *= float(self.get_parameter('linear_scale_x').value)
            vy_body *= float(self.get_parameter('linear_scale_y').value)

            odom_msg = Odometry()
            odom_msg.header.stamp = imu_msg.header.stamp
            odom_msg.header.frame_id = self.odom_frame_id
            odom_msg.child_frame_id  = self.base_frame_id
            odom_msg.pose.pose.position.x = o_x
            odom_msg.pose.pose.position.y = o_y
            odom_msg.pose.pose.position.z = o_z
            if ekf_quat_present and (o_qx or o_qy or o_qz or o_qw):
                odom_msg.pose.pose.orientation = Quaternion(
                    x=o_qx, y=o_qy, z=o_qz, w=o_qw)
            else:
                odom_msg.pose.pose.orientation = _yaw_to_quaternion(o_phi)
            odom_msg.twist.twist.linear.x  = vx_body
            odom_msg.twist.twist.linear.y  = vy_body
            odom_msg.twist.twist.linear.z  = 0.0
            # Negate: the wheel/firmware-derived yaw rate (o_w) has the opposite
            # sign convention to REP-103 (the IMU / cmd_vel convention). Nav2 MPPI
            # uses this field as velocity feedback (odom_topic=/odrive/odom), so a
            # flipped sign breaks its yaw control (slow/non-converging final rotation).
            # The EKF ignores this field (odom0_config vyaw=False; yaw rate from IMU).
            odom_msg.twist.twist.angular.z = -o_w

            _LARGE = 1.0e9
            _pose_cov = [0.0] * 36
            _twist_cov = [0.0] * 36
            if ekf_cov_present:
                _pose_cov[0]  = o_var_x      # x
                _pose_cov[7]  = o_var_y      # y
                _pose_cov[14] = _LARGE       # z   (not estimated)
                _pose_cov[21] = _LARGE       # roll
                _pose_cov[28] = _LARGE       # pitch
                _pose_cov[35] = o_var_yaw    # yaw
                _twist_cov[0]  = o_var_vx    # vx
                _twist_cov[7]  = o_var_vy    # vy
                _twist_cov[14] = _LARGE      # vz
                _twist_cov[21] = _LARGE      # wx
                _twist_cov[28] = _LARGE      # wy
                _twist_cov[35] = o_var_wz    # wz
            else:
                # Legacy firmware — no covariance emitted. Keep the previous
                # small placeholders so consumers see a positive-definite
                # diagonal. Mecanum lateral rollers slip more than the
                # longitudinal ones, so trust vy less than vx.
                _pose_cov[0] = _pose_cov[7] = 0.05; _pose_cov[35] = 0.10
                _twist_cov[0] = 0.02; _twist_cov[7] = 0.08; _twist_cov[35] = 0.05
            odom_msg.pose.covariance  = _pose_cov
            odom_msg.twist.covariance = _twist_cov
            self.pub_odom.publish(odom_msg)

            # ── TF: odom -> base_link ──────────────────────────────────────
            # Mirrors robot_localization's `publish_tf: True` for the omnibase
            # config (omni_basics.launch.py). The orientation is the same
            # quaternion that just went into the odom message so the TF tree
            # and the message agree exactly. Disable at runtime via
            # `publish_tf:=False` if you ever want to run an external EKF
            # alongside.
            if self.publish_tf:
                tf_msg = TransformStamped()
                tf_msg.header.stamp = odom_msg.header.stamp
                tf_msg.header.frame_id = self.odom_frame_id
                tf_msg.child_frame_id  = self.base_frame_id
                tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
                tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
                tf_msg.transform.translation.z = odom_msg.pose.pose.position.z
                tf_msg.transform.rotation      = odom_msg.pose.pose.orientation
                self._tf_broadcaster.sendTransform(tf_msg)
            self.pub_node_ids   .publish(_i32(node_ids))
            self.pub_axis_errors.publish(_i32(axis_errors))
            self.pub_axis_states.publish(_i32(axis_states))
            self.pub_ctrl_status.publish(_i32(ctrl_status))
            self.pub_updated    .publish(_i32(updated))
            self.pub_pos        .publish(_f32(pos_est))
            self.pub_vel        .publish(_f32(vel_est))
            self.pub_shadow     .publish(_i32(shadows))
            self.pub_cpr        .publish(_i32(cprs))
            self.pub_vbus       .publish(_f32(vbus))
            self.pub_ibus       .publish(_f32(ibus))
            self.pub_iq_set     .publish(_f32(iq_set))
            self.pub_iq_meas    .publish(_f32(iq_meas))

            for nid in node_ids:
                if nid > 0 and nid not in self.discovered_node_ids:
                    self.discovered_node_ids.append(nid)
                    self.get_logger().info(f"Discovered node_id={nid}")
            disc = Int32MultiArray(); disc.data = list(self.discovered_node_ids)
            self.pub_discovered.publish(disc)

            lines = []
            for j in range(4):
                sn = AXIS_STATES.get(axis_states[j], f"STATE_{axis_states[j]}")
                cn = CONTROL_MODES.get(ctrl_status[j], str(ctrl_status[j]))
                lines.append(
                    f"  Axis {j} (node {node_ids[j]}): {sn} | {cn} | "
                    f"err={axis_errors[j]:#010x} | "
                    f"pos={pos_est[j]:.3f} | vel={vel_est[j]:.3f} | "
                    f"Vbus={vbus[j]:.2f}V")
            self.pub_states_str.publish(String(data="\n".join(lines)))
            self.pub_debug     .publish(String(data="\n".join(lines)))

            # Firmware SM state — index 96 is the last field in both slim and
            # fat frames. A truncated serial read drops it first. When absent,
            # preserve the last known state so the dashboard pill doesn't
            # momentarily flash UNKNOWN on the re-arm gap after ESTOP release.
            sm_raw = data.get('SM_state')
            if sm_raw is not None:
                sm_state = int(sm_raw)
                sm_label = SM_STATE_LABELS.get(sm_state, 'UNKNOWN')
            else:
                sm_state = self._latest_telem.get('sm_state')
                sm_label = self._latest_telem.get('sm_state_label', 'UNKNOWN')

            telem = {
                'cmd_vx': cmd_vx, 'cmd_vy': cmd_vy, 'cmd_wz': cmd_wz,
                'imu_yaw': imu_y, 'imu_roll': imu_r, 'imu_pitch': imu_p,
                'imu_qx': imu_qx, 'imu_qy': imu_qy,
                'imu_qz': imu_qz, 'imu_qw': imu_qw,
                'imu_wx': imu_wx, 'imu_wy': imu_wy, 'imu_wz': imu_wz,
                'imu_ax': imu_ax, 'imu_ay': imu_ay, 'imu_az': imu_az,
                'ik_speeds': ik,
                'odom_phi': o_phi, 'odom_x': o_x, 'odom_y': o_y,
                'odom_w': o_w, 'odom_vx': o_vx, 'odom_vy': o_vy,
                'node_ids': node_ids, 'axis_errors': axis_errors,
                'axis_states': axis_states, 'ctrl_status': ctrl_status,
                'updated': updated, 'pos_est': pos_est, 'vel_est': vel_est,
                'bus_voltage': vbus, 'bus_current': ibus,
                'iq_setpoint': iq_set, 'iq_measured': iq_meas,
                'bt_active': bt_active,
                'sm_state': sm_state, 'sm_state_label': sm_label,
            }
            # Fold the current link ages into the dict so the per-packet
            # emit is consistent with what the 4 Hz timer would publish.
            self._inject_link_ages(telem)
            self._latest_telem = telem
            if self._sio:
                self._sio.emit('telemetry', telem)

        except Exception as e:
            self.get_logger().error(f"Parse error: {e}")

    # ── utilities ─────────────────────────────────────────────────────────────

    @staticmethod
    def _empty_telem() -> dict:
        z4f = [0.0]*4; z4i = [0]*4
        return dict(cmd_vx=0.0, cmd_vy=0.0, cmd_wz=0.0,
                    imu_yaw=0.0, imu_roll=0.0, imu_pitch=0.0,
                    imu_qx=0.0, imu_qy=0.0, imu_qz=0.0, imu_qw=1.0,
                    imu_wx=0.0, imu_wy=0.0, imu_wz=0.0,
                    imu_ax=0.0, imu_ay=0.0, imu_az=0.0,
                    ik_speeds=z4f, odom_phi=0.0, odom_x=0.0, odom_y=0.0,
                    odom_w=0.0, odom_vx=0.0, odom_vy=0.0,
                    node_ids=z4i, axis_errors=z4i, axis_states=z4i,
                    ctrl_status=z4i, updated=z4i,
                    pos_est=z4f, vel_est=z4f,
                    bus_voltage=z4f, bus_current=z4f,
                    iq_setpoint=z4f, iq_measured=z4f,
                    bt_active=0,
                    # Link ages start as "unknown" — the first periodic tick
                    # or first received line will overwrite these.
                    esp32_age_ms=None, esp32_status='UNKNOWN',
                    pc_age_ms=None,    pc_status='UNKNOWN',
                    sm_state=None,     sm_state_label='UNKNOWN')

    @staticmethod
    def _sf(v) -> float:
        try:
            x = float(v) if v is not None else 0.0
            return x if math.isfinite(x) else 0.0
        except Exception:
            return 0.0

    @staticmethod
    def _si(v) -> int:
        try:
            return max(-2_147_483_648, min(2_147_483_647,
                       int(float(v)) if v is not None else 0))
        except Exception:
            return 0


def main(args=None):
    rclpy.init(args=args)
    node = ODriveDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
