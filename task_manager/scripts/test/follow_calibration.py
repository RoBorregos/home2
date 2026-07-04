#!/usr/bin/env python3

"""Follow-person CALIBRATION run — measures the whole pipeline while you walk.

Drives the same services the HRIC FOLLOW_PERSON state uses (tracker, arm
follow, nav follow), then live-monitors every link of the chain and prints a
summary with rule-based tuning hints when you stop:

    tracker centroid rate / gaps   -> is the tracker fast enough?
    3D point (RESULTS) rate / gaps -> is depth gating dropping publishes?
    arm centroid error + flips     -> PID lag vs oscillation (kp/kd)
    joint1 range dwell             -> is the arm living at its limits?
    /goal_update rate              -> is the smoother feeding Nav2?
    /cmd_vel saturation            -> is the base speed cap the bottleneck?

Stages (pick what you're calibrating; earlier stages isolate subsystems):
    arm  — tracker + arm follow only (robot base stays still; nav not needed)
    base — tracker + nav follow only (arm stays put)
    full — everything, the real HRIC behavior

Usage (nav+vision+manipulation stacks up first):
    ros2 run task_manager follow_calibration.py
    ros2 run task_manager follow_calibration.py --ros-args -p stage:=arm

Live-tune while it runs (params are read every control tick):
    ros2 param set /follow_person_controller kp 2.0
    ros2 param set /follow_person_controller kd 0.2
    ros2 param set /follow_person_controller max_velocity 1.0
    ros2 param set /person_goal_smoother lead_time 0.6
    ros2 param set /person_goal_smoother follow_distance 0.8

Type 'stop' (or Ctrl-C) to end the run: follow/tracking are always shut off
and the summary is printed.
"""

import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from frida_interfaces.srv import FollowFace
from frida_constants.vision_constants import (
    CENTROID_TOPIC,
    RESULTS_TOPIC,
    SET_TARGET_TOPIC,
)
from frida_constants.navigation_constants import (
    FOLLOW_PERSON_NAV_SERVICE,
    GOAL_UPDATE_TOPIC,
)

ARM_FOLLOW_SERVICE = "/follow_person"
STOP_WORDS = {"stop", "s", "q", "quit", "exit"}
SERVICE_TIMEOUT = 10.0
# Thresholds the hints are judged against (matched to the current defaults of
# tracker_node / person_goal_smoother / follow_person_controller).
GOOD_CENTROID_HZ = 8.0
RESULTS_VS_CENTROID = 0.6      # results rate should be >= this x centroid rate
SMOOTHER_TIMEOUT_S = 2.0       # person_goal_smoother 'timeout' default
ARM_ERR_LAGGY = 0.25           # mean |centroid| above this = arm lagging
FLIPS_OSCILLATING = 1.5        # error sign flips/s above this = oscillating
J1_MIN, J1_MAX, J1_MARGIN = -3.05, -0.2, 0.35  # follow_person_controller defaults
# Fallback only: the real cap is read from controller_server (FollowPath.vx_max)
# once follow mode is active, so the saturation hint is judged against whatever
# profile is actually loaded (healthy nav2_omni_following vs the 3-wheel limp one).
BASE_VX_CAP = 0.5


class TopicStat:
    """Rate/gap bookkeeping for one topic."""

    def __init__(self):
        self.count = 0
        self.first_t = None
        self.last_t = None
        self.max_gap = 0.0
        self.dropouts = 0  # gaps > 0.4 s while data was flowing

    def tick(self):
        now = time.monotonic()
        if self.last_t is not None:
            gap = now - self.last_t
            self.max_gap = max(self.max_gap, gap)
            if gap > 0.4:
                self.dropouts += 1
        else:
            self.first_t = now
        self.last_t = now
        self.count += 1

    def rate(self):
        if self.count < 2 or self.first_t is None or self.last_t is None:
            return 0.0
        span = self.last_t - self.first_t
        return (self.count - 1) / span if span > 0 else 0.0


class FollowCalibration(Node):
    def __init__(self):
        super().__init__("follow_calibration")
        stage = self.declare_parameter("stage", "full").value
        if stage not in ("arm", "base", "full"):
            self.get_logger().warn(f"Unknown stage '{stage}', using 'full'")
            stage = "full"
        self.stage = stage
        self.use_arm = stage in ("arm", "full")
        self.use_nav = stage in ("base", "full")

        # --- Metrics ---
        self.centroid = TopicStat()
        self.results = TopicStat()
        self.goals = TopicStat()
        self.err_abs_sum = 0.0
        self.err_samples = []
        self.err_flips = 0
        self._last_err_sign = 0
        self.j1_samples = 0
        self.j1_near_limit = 0
        self.j1_lo_seen = math.inf
        self.j1_hi_seen = -math.inf
        self.dist_samples = []
        self.vx_max_seen = 0.0
        self.wz_max_seen = 0.0
        self.vx_sat_ticks = 0
        self.cmd_ticks = 0
        self.base_vx_cap = BASE_VX_CAP

        # --- Subscribers (monitor only) ---
        self.create_subscription(Point, CENTROID_TOPIC, self._centroid_cb, 10)
        self.create_subscription(PointStamped, RESULTS_TOPIC, self._results_cb, 10)
        self.create_subscription(PoseStamped, GOAL_UPDATE_TOPIC, self._goal_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._joints_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        # --- Service clients (same contracts the HRIC TM uses) ---
        self.track_client = self.create_client(SetBool, SET_TARGET_TOPIC)
        self.nav_client = self.create_client(SetBool, FOLLOW_PERSON_NAV_SERVICE)
        self.arm_client = self.create_client(FollowFace, ARM_FOLLOW_SERVICE)
        self.ctrl_param_client = self.create_client(
            GetParameters, "/controller_server/get_parameters"
        )

        self.create_timer(2.0, self._live_line)
        self._started = time.monotonic()

    # -- topic callbacks ------------------------------------------------

    def _centroid_cb(self, msg: Point):
        self.centroid.tick()
        self.err_abs_sum += abs(msg.x)
        self.err_samples.append(abs(msg.x))
        sign = 1 if msg.x > 0.03 else (-1 if msg.x < -0.03 else 0)
        if sign != 0 and self._last_err_sign != 0 and sign != self._last_err_sign:
            self.err_flips += 1
        if sign != 0:
            self._last_err_sign = sign

    def _results_cb(self, msg: PointStamped):
        self.results.tick()
        d = math.sqrt(msg.point.x**2 + msg.point.y**2 + msg.point.z**2)
        if math.isfinite(d):
            self.dist_samples.append(d)

    def _goal_cb(self, _msg: PoseStamped):
        self.goals.tick()

    def _joints_cb(self, msg: JointState):
        if "joint1" not in msg.name:
            return
        j1 = msg.position[msg.name.index("joint1")]
        self.j1_samples += 1
        self.j1_lo_seen = min(self.j1_lo_seen, j1)
        self.j1_hi_seen = max(self.j1_hi_seen, j1)
        if j1 < J1_MIN + J1_MARGIN or j1 > J1_MAX - J1_MARGIN:
            self.j1_near_limit += 1

    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_ticks += 1
        vx = math.hypot(msg.linear.x, msg.linear.y)
        self.vx_max_seen = max(self.vx_max_seen, vx)
        self.wz_max_seen = max(self.wz_max_seen, abs(msg.angular.z))
        if vx >= 0.95 * self.base_vx_cap:
            self.vx_sat_ticks += 1

    # -- live status -----------------------------------------------------

    def _live_line(self):
        if self.centroid.count == 0 and self.results.count == 0:
            print("   ... waiting for tracker data (is a person in view?)")
            return
        dist = self.dist_samples[-1] if self.dist_samples else float("nan")
        print(
            f"   centroid {self.centroid.rate():4.1f} Hz | "
            f"3D {self.results.rate():4.1f} Hz | "
            f"goals {self.goals.rate():4.1f} Hz | "
            f"person at {dist:4.2f} m | "
            f"|err| {self._mean_err():.2f} | flips {self.err_flips}"
        )

    def _mean_err(self):
        return self.err_abs_sum / self.centroid.count if self.centroid.count else 0.0

    # -- service helpers --------------------------------------------------

    def _call(self, client, request, label):
        """call_async + wait via done-event; safe with the background executor
        (spin_until_future_complete would fight it for the node)."""
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            print(f"   !! {label}: service {client.srv_name} not available")
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=SERVICE_TIMEOUT):
            print(f"   !! {label}: no response after {SERVICE_TIMEOUT}s")
            return None
        return future.result()

    def set_tracking(self, on: bool) -> bool:
        res = self._call(self.track_client, SetBool.Request(data=on), "tracker")
        return bool(res and res.success)

    def set_nav_follow(self, on: bool) -> bool:
        res = self._call(self.nav_client, SetBool.Request(data=on), "nav follow")
        return bool(res and res.success)

    def set_arm_follow(self, on: bool) -> bool:
        req = FollowFace.Request()
        req.follow_face = on
        res = self._call(self.arm_client, req, "arm follow")
        return bool(res and res.success)

    def fetch_vx_cap(self):
        """Read FollowPath.vx_max off the live controller_server. Called AFTER
        follow mode is on, so the smoother has already pushed the follow overlay
        and this returns the cap that actually binds during the run."""
        req = GetParameters.Request(names=["FollowPath.vx_max"])
        res = self._call(self.ctrl_param_client, req, "controller vx cap")
        if res and res.values and res.values[0].type == 3:  # PARAMETER_DOUBLE
            self.base_vx_cap = res.values[0].double_value
            print(f"   active base vx cap: {self.base_vx_cap:.2f} m/s")
        else:
            print(f"   !! could not read FollowPath.vx_max — "
                  f"assuming {self.base_vx_cap:.2f} m/s for the saturation hint")

    # -- summary ----------------------------------------------------------

    def summary(self):
        span = time.monotonic() - self._started
        mean_err = self._mean_err()
        p95 = 0.0
        if self.err_samples:
            s = sorted(self.err_samples)
            p95 = s[min(len(s) - 1, int(0.95 * len(s)))]
        flips_per_s = self.err_flips / span if span > 0 else 0.0
        limit_frac = self.j1_near_limit / self.j1_samples if self.j1_samples else 0.0
        c_rate, r_rate = self.centroid.rate(), self.results.rate()

        print("\n=== Follow calibration summary "
              f"(stage={self.stage}, {span:.0f}s) ===")
        print(f"  tracker centroid : {c_rate:5.1f} Hz   max gap {self.centroid.max_gap:.2f}s  dropouts {self.centroid.dropouts}")
        print(f"  3D points        : {r_rate:5.1f} Hz   max gap {self.results.max_gap:.2f}s  dropouts {self.results.dropouts}")
        print(f"  nav goal updates : {self.goals.rate():5.1f} Hz   max gap {self.goals.max_gap:.2f}s")
        if self.dist_samples:
            ds = sorted(self.dist_samples)
            print(f"  person distance  : median {ds[len(ds)//2]:.2f} m   max {ds[-1]:.2f} m")
        print(f"  arm error |cx|   : mean {mean_err:.2f}   p95 {p95:.2f}   sign flips {flips_per_s:.2f}/s")
        if self.j1_samples:
            print(f"  joint1 range     : [{self.j1_lo_seen:.2f}, {self.j1_hi_seen:.2f}] rad   near-limit {100 * limit_frac:.0f}% of time")
        if self.cmd_ticks:
            print(f"  base velocity    : max {self.vx_max_seen:.2f} m/s (cap {self.base_vx_cap})   "
                  f"saturated {100 * self.vx_sat_ticks / self.cmd_ticks:.0f}% of ticks   max |wz| {self.wz_max_seen:.2f}")

        print("\n--- Tuning hints ---")
        hints = []
        if self.centroid.count == 0:
            hints.append("No centroid data at all: tracker not running / no target locked.")
        else:
            if c_rate < GOOD_CENTROID_HZ:
                hints.append(
                    f"Tracker slow ({c_rate:.1f} Hz < {GOOD_CENTROID_HZ}): check GPU load "
                    "(jtop) and the tracker's 'Det+Tracking took' log."
                )
            if self.results.count and r_rate < RESULTS_VS_CENTROID * c_rate:
                hints.append(
                    "3D rate well below centroid rate: depth pairing/jump gates are "
                    "dropping publishes — check ZED depth mode and tracker warns."
                )
            if self.results.max_gap > SMOOTHER_TIMEOUT_S:
                hints.append(
                    f"3D gap {self.results.max_gap:.1f}s exceeds the smoother timeout "
                    f"({SMOOTHER_TIMEOUT_S}s): the lost-person recovery WILL trigger "
                    "mid-follow. Find the gap source before tuning gains."
                )
            if mean_err > ARM_ERR_LAGGY and flips_per_s < FLIPS_OSCILLATING:
                hints.append(
                    f"Arm lags the person (mean |err| {mean_err:.2f}): raise kp "
                    "(ros2 param set /follow_person_controller kp <val>) or max_velocity."
                )
            if flips_per_s > FLIPS_OSCILLATING:
                hints.append(
                    f"Arm oscillates ({flips_per_s:.1f} flips/s): lower kp or raise kd."
                )
            if limit_frac > 0.2:
                hints.append(
                    "joint1 near a limit >20% of the time: the base isn't rotating "
                    "toward the person — check nav goal yaw / raise wz_max."
                )
            if self.cmd_ticks and self.vx_sat_ticks / self.cmd_ticks > 0.5:
                hints.append(
                    "Base at its speed cap >50% of the time: person is faster than "
                    "the base — raise vx/vy in the ACTIVE follow overlay "
                    "(nav2_omni_limp_following.yaml while on 3 wheels, else "
                    "nav2_omni_following.yaml; test safety!) or accept a larger "
                    "follow_distance / ask the person to slow down."
                )
        if not hints:
            hints.append("Everything within thresholds — save these params as defaults.")
        for h in hints:
            print(f"  * {h}")
        print()


def main(args=None):
    rclpy.init(args=args)
    node = FollowCalibration()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    tracking = nav_on = arm_on = False
    print(f"\n=== Follow calibration (stage={node.stage}) ===")
    print("  Stand in front of the robot, then walk around when follow starts.")
    print("  Type 'stop' + Enter (or Ctrl-C) to finish and get the summary.\n")
    try:
        print("-> Locking tracker on the person in view...")
        tracking = node.set_tracking(True)
        if not tracking:
            print("   Could not lock a target — is tracker_node up and a person visible?")
            return
        if node.use_nav:
            print("-> Starting nav follow...")
            nav_on = node.set_nav_follow(True)
            if not nav_on:
                print("   Nav follow failed to start; continuing without base motion.")
            else:
                node.fetch_vx_cap()
        if node.use_arm:
            print("-> Starting arm follow...")
            arm_on = node.set_arm_follow(True)
            if not arm_on:
                print("   Arm follow failed to start; continuing without arm pan.")
        print("Calibration running — walk normal speed, cut corners, cross rooms.\n")
        while True:
            try:
                cmd = input()
            except EOFError:
                break
            if cmd.strip().lower() in STOP_WORDS:
                break
    except KeyboardInterrupt:
        print("\n(Ctrl-C) stopping...")
    finally:
        if nav_on:
            node.set_nav_follow(False)
        if arm_on:
            node.set_arm_follow(False)
        if tracking:
            node.set_tracking(False)
        node.summary()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
