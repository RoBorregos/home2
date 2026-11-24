#!/usr/bin/env python3
"""Person-following scenarios in the navigation sim.

Drives the real follow pipeline (tracker stand-in -> person_goal_smoother ->
Nav2 GoalUpdater BT, started through nav_central's follow_person service exactly
as the task managers do) against a simulated walking person, and grades every
scenario on ground truth: the robot's Gazebo pose vs the person's, the follow
status reported on /navigation/follow_status, and the arena walls.

Needs a running `--nav` sim plus sim_person.py and sim_person_tracker.py
(`./run.sh simulation --follow-test` starts them). Results land in
/workspace/sim_logs/follow_test_results.json and follow_<scenario>_poses.csv.

    ros2 run frida_gz_sim sim_follow_test.py
    ros2 run frida_gz_sim sim_follow_test.py --ros-args -p scenarios:="straight_walk,corner_turn"
    ros2 run frida_gz_sim sim_follow_test.py --ros-args -p tag:=baseline
"""

import csv
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from frida_constants.navigation_constants import (
    INITIAL_POSE_TOPIC,
    FOLLOW_STATUS_TOPIC,
    GO_TO_POSE_SERVICE,
    GOAL_UPDATE_TOPIC,
)
from frida_constants.vision_constants import SET_TARGET_TOPIC
from frida_gz_sim import ground_truth as gt
from frida_gz_sim.nav import MAP_NAME
from frida_interfaces.srv import GoToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool
from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.utils.status import Status
from task_manager.utils.task import Task

LOGS = "/workspace/sim_logs"
STAGING_TIMEOUT = 300.0
SAMPLE_DT = 0.1

# Corridor y=-9.6, free x -2.6..4.4; wall at x~1.3 hides the laundry door (corner loss).
ROBOT_START = (-2.0, -9.6, 0.0)
PERSON_START = (-0.5, -9.6)
HIDEOUT = (2.4, -7.4)

# waypoints: [x, y, speed_mps, dwell_s]; speed 0 = teleport
SCENARIOS = {
    "straight_walk": {
        "route": [[3.8, -9.6, 0.8, 8.0]],
        "duration": 22.0,
        "max_gap": 3.0,
        "end_gap": (0.5, 1.8),
        "max_lost_s": 2.0,
    },
    "fast_walk": {
        "route": [[3.8, -9.6, 1.2, 12.0]],
        "duration": 22.0,
        "max_gap": 4.5,
        "end_gap": (0.5, 1.8),
        "catch_up_s": 10.0,
    },
    "stop_and_go": {
        "route": [[1.3, -9.6, 0.8, 5.0], [3.8, -9.6, 0.8, 8.0]],
        "duration": 26.0,
        "max_gap": 3.0,
        "end_gap": (0.5, 1.8),
        "max_lost_s": 2.0,
        "still_drift": 0.35,
    },
    "corner_turn": {
        "route": [[2.4, -9.6, 0.8, 0.0], [2.4, -7.6, 0.8, 0.0], [3.6, -7.6, 0.6, 12.0]],
        "duration": 30.0,
        "max_gap": 4.5,
        "end_gap": (0.5, 2.0),
        "recover_s": 12.0,
    },
    "walk_back": {
        "route": [
            [-1.5, -9.6, 0.6, 2.0],
            [-2.4, -8.5, 0.6, 0.0],
            [-2.6, -8.3, 0.4, 8.0],
        ],
        "person_start": (0.8, -9.6),
        "duration": 22.0,
        "max_gap": 3.0,
        "end_gap": (0.4, 2.0),
        "min_gap": 0.3,
    },
    "lost_then_return": {
        "route": [
            [1.0, -9.6, 0.8, 1.0],
            [HIDEOUT[0], HIDEOUT[1], 0.0, 9.0],
            [0.3, -9.6, 0.0, 10.0],
        ],
        "duration": 30.0,
        "max_gap": 99.0,
        "end_gap": (0.5, 2.0),
        "expect_states": ["lost"],
        "recover_s": 10.0,
    },
    "stop_service": {
        "route": [[1.5, -9.6, 0.8, 30.0]],
        "duration": 6.0,
        "max_gap": 99.0,
        "end_gap": (0.0, 99.0),
        "stop_check": True,
    },
}


def _plain(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


class SimFollowTest(Node):
    def __init__(self):
        super().__init__("sim_follow_test")
        names = self.declare_parameter("scenarios", "").value
        self.names = [n for n in names.split(",") if n] if names else list(SCENARIOS)
        self.tag = self.declare_parameter("tag", "").value
        self.repeat = self.declare_parameter("repeat", 1).value
        self.navigation = NavigationTasks(self, task=Task.DEBUG)
        self.pose_client = self.create_client(GoToPose, GO_TO_POSE_SERVICE)
        self.track_client = self.create_client(SetBool, SET_TARGET_TOPIC)
        self.param_client = self.create_client(
            GetParameters, "/controller_server/get_parameters"
        )
        self.route_pub = self.create_publisher(String, "/sim/person/route", 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, INITIAL_POSE_TOPIC, 10
        )
        self.gz = GzNode()

        maps = f"{get_package_share_directory('map_context')}/maps"
        with open(f"{maps}/{MAP_NAME}.yaml") as f:
            meta = yaml.safe_load(f)
        self.walls = gt.map_obstacle_points(
            f"{maps}/{meta['image']}",
            meta["resolution"],
            meta["origin"],
            meta["occupied_thresh"],
        )

        # Ground truth: gz odometry is in world frame, which equals map.
        self.robot = None
        self.person = None
        self.status = "unknown"
        self.status_log = []
        self.goal_updates = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(PoseStamped, "/sim/person/pose", self._person_cb, 20)
        self.create_subscription(String, FOLLOW_STATUS_TOPIC, self._status_cb, latched)
        self.create_subscription(PoseStamped, GOAL_UPDATE_TOPIC, self._goal_cb, 10)
        self.results = []

    # ------------------------------------------------------------ inputs
    def _odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.robot = (p.x, p.y, yaw)

    def _person_cb(self, msg):
        self.person = (msg.pose.position.x, msg.pose.position.y)

    def _status_cb(self, msg):
        if msg.data != self.status:
            self.status_log.append((time.time(), msg.data))
        self.status = msg.data

    def _goal_cb(self, msg):
        self.goal_updates += 1

    def wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---------------------------------------------------------- helpers
    def event(self, name, what):
        self.get_logger().info(f"[{name}] {what}")

    def go_to(self, x, y, yaw):
        req = GoToPose.Request()
        req.target_pose.header.frame_id = "map"
        req.target_pose.pose.position.x, req.target_pose.pose.position.y = x, y
        req.target_pose.pose.orientation.z = math.sin(yaw / 2)
        req.target_pose.pose.orientation.w = math.cos(yaw / 2)
        if not self.pose_client.wait_for_service(timeout_sec=10.0):
            return False, "go_to_pose unavailable"
        future = self.pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=STAGING_TIMEOUT)
        res = future.result()
        if res is None:
            return False, "staging timed out"
        return res.success, res.error

    def teleport(self, x, y, yaw):
        """Put the robot at (x, y, yaw) in Gazebo and re-seed slam_toolbox there.

        Driving across the arena between scenarios is slow and leaves the robot
        wedged against walls after a bad run; teleporting makes every scenario
        start from the same pose. slam_toolbox re-localizes on /initialpose, so
        the estimate follows the jump."""
        req = Pose()
        req.name = "frida"
        req.position.x, req.position.y, req.position.z = float(x), float(y), 0.02
        req.orientation.z, req.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        ok, _ = self.gz.request(
            f"/world/arena_{MAP_NAME}/set_pose", req, Pose, Boolean, 3000
        )
        if not ok:
            return False, "gz set_pose failed"
        self.wait(1.0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05
        cov[35] = 0.03
        msg.pose.covariance = cov
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            self.wait(0.5)
        self.wait(3.0)
        if self.robot is None or math.hypot(self.robot[0] - x, self.robot[1] - y) > 0.1:
            return False, f"robot not at start after teleport: {self.robot}"
        return True, ""

    def track(self, on):
        if not self.track_client.wait_for_service(timeout_sec=5.0):
            return False
        future = self.track_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result() is not None and future.result().success

    def send_route(self, waypoints):
        self.route_pub.publish(String(data=json.dumps({"waypoints": waypoints})))

    def place_person(self, x, y):
        self.send_route([[x, y, 0.0, 0.0]])
        self.wait(0.5)

    def vx_max(self):
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            return None
        future = self.param_client.call_async(
            GetParameters.Request(names=["FollowPath.vx_max"])
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res is None or not res.values:
            return None
        return res.values[0].double_value

    def clearance(self):
        rx, ry, ryaw = self.robot
        near = self.walls[np.hypot(self.walls[:, 0] - rx, self.walls[:, 1] - ry) < 1.5]
        if len(near) == 0:
            return 9.0
        return float(np.min(gt.footprint_clearance(gt.to_base(near, rx, ry, ryaw))))

    # ---------------------------------------------------------- scenario
    def run_one(self, name):
        sc = SCENARIOS[name]
        self.event(name, "staging")
        self.navigation.follow_person(False)
        self.track(False)
        self.place_person(*sc.get("person_start", PERSON_START))
        ok, err = self.teleport(*ROBOT_START)
        if not ok:
            return {"name": name, "passed": False, "error": f"staging: {err}"}
        self.wait(1.5)
        vx_standard = self.vx_max()
        if not self.track(True):
            return {"name": name, "passed": False, "error": "tracker could not lock"}
        self.status_log = []
        self.goal_updates = 0
        t0 = time.time()
        status, msg = self.navigation.follow_person(True)
        start_latency = time.time() - t0
        if status != Status.EXECUTION_SUCCESS:
            return {"name": name, "passed": False, "error": f"follow_person: {msg}"}
        self.wait(1.0)
        self.send_route(sc["route"])
        self.event(name, f"person walking, follow started in {start_latency:.1f}s")

        samples = []
        stop_at = None
        stopped_pose = None
        end = t0 + sc["duration"]
        while time.time() < end:
            self.wait(SAMPLE_DT)
            if self.robot is None or self.person is None:
                continue
            t = time.time() - t0
            gap = math.hypot(
                self.robot[0] - self.person[0], self.robot[1] - self.person[1]
            )
            samples.append(
                (t, *self.robot, *self.person, gap, self.status, self.clearance())
            )
            if sc.get("stop_check") and stop_at is None and t > 4.0:
                self.navigation.follow_person(False)
                stop_at = time.time()
                self.wait(2.0)
                stopped_pose = self.robot
                self.wait(2.0)
                end = time.time() + 0.5

        res = {
            "name": name,
            "start_latency_s": round(start_latency, 2),
            "goal_updates": self.goal_updates,
        }
        if not sc.get("stop_check"):
            self.navigation.follow_person(False)
        self.track(False)
        self.wait(1.0)
        self._save_csv(name, samples)
        res.update(self.grade(name, sc, samples, stopped_pose, vx_standard))
        self.get_logger().info(
            f"[{name}] {'PASS' if res['passed'] else 'FAIL'} {json.dumps(res, default=_plain)}"
        )
        return res

    def grade(self, name, sc, samples, stopped_pose, vx_standard):
        if not samples:
            return {"passed": False, "error": "no samples"}
        arr = np.array(
            [s[:7] for s in samples], dtype=float
        )  # t, rx, ry, ryaw, px, py, gap
        gaps = arr[:, 6]
        statuses = [s[7] for s in samples]
        clear = np.array([s[8] for s in samples])
        checks = {}
        metrics = {
            "gap_max": round(float(gaps.max()), 2),
            "gap_mean": round(float(gaps.mean()), 2),
            "gap_end": round(float(gaps[-1]), 2),
            "clearance_min": round(float(clear.min()), 2),
            "states": [s for _, s in self.status_log],
        }
        # seconds spent not following (lost/searching/gave_up), longest streak
        streak = longest = 0.0
        for i in range(1, len(samples)):
            if statuses[i] in ("lost", "searching", "gave_up"):
                streak += samples[i][0] - samples[i - 1][0]
                longest = max(longest, streak)
            else:
                streak = 0.0
        metrics["lost_longest_s"] = round(longest, 1)
        checks["no_collision"] = bool(clear.min() > 0.0)
        checks["gap_max"] = bool(gaps.max() <= sc["max_gap"])
        lo, hi = sc["end_gap"]
        checks["end_gap"] = bool(lo <= gaps[-1] <= hi)
        if "max_lost_s" in sc:
            checks["never_lost"] = bool(longest <= sc["max_lost_s"])
        if "min_gap" in sc:
            checks["kept_distance"] = bool(gaps.min() >= sc["min_gap"])
            metrics["gap_min"] = round(float(gaps.min()), 2)
        if "catch_up_s" in sc:
            # person stops at the first waypoint; the robot must close to < 1.8 m within catch_up_s
            dist = np.hypot(
                arr[:, 4] - sc["route"][0][0], arr[:, 5] - sc["route"][0][1]
            )
            arrived = np.nonzero(dist < 0.05)[0]
            if len(arrived):
                t_stop = arr[arrived[0], 0]
                close = np.nonzero((arr[:, 0] > t_stop) & (gaps < 1.8))[0]
                metrics["catch_up_s"] = (
                    round(float(arr[close[0], 0] - t_stop), 1) if len(close) else None
                )
                checks["catch_up"] = bool(
                    len(close) and arr[close[0], 0] - t_stop <= sc["catch_up_s"]
                )
            else:
                checks["catch_up"] = False
        if "still_drift" in sc:
            # while the person dwells at the first waypoint the robot must settle, not oscillate
            wx, wy, _, dwell = sc["route"][0]
            dist = np.hypot(arr[:, 4] - wx, arr[:, 5] - wy)
            idx = np.nonzero(dist < 0.05)[0]
            if len(idx) > 10:
                t_stop = arr[idx[0], 0]
                win = arr[(arr[:, 0] > t_stop + 3.0) & (arr[:, 0] < t_stop + dwell)]
                drift = (
                    float(
                        np.hypot(win[:, 1] - win[-1, 1], win[:, 2] - win[-1, 2]).max()
                    )
                    if len(win)
                    else 0.0
                )
                metrics["still_drift"] = round(drift, 2)
                checks["settles_when_person_stops"] = drift <= sc["still_drift"]
            else:
                checks["settles_when_person_stops"] = False
        if "expect_states" in sc:
            seen = [s for _, s in self.status_log]
            checks["went_through_lost"] = all(st in seen for st in sc["expect_states"])
        if "recover_s" in sc:
            # after the first loss, following must resume within recover_s
            t_lost = next((t for t, s in self.status_log if s == "lost"), None)
            t_back = next(
                (
                    t
                    for t, s in self.status_log
                    if s == "following" and t_lost and t > t_lost
                ),
                None,
            )
            if t_lost is None:
                metrics["recover_s"] = "never lost"
                checks["recovered"] = True
            else:
                metrics["recover_s"] = round(t_back - t_lost, 1) if t_back else None
                checks["recovered"] = bool(
                    t_back and t_back - t_lost <= sc["recover_s"]
                )
        if sc.get("stop_check"):
            moved = (
                math.hypot(
                    self.robot[0] - stopped_pose[0], self.robot[1] - stopped_pose[1]
                )
                if stopped_pose
                else 9.0
            )
            metrics["moved_after_stop"] = round(moved, 2)
            checks["stopped"] = moved < 0.05
            after = self.vx_max()
            metrics["vx_max_standard"] = vx_standard
            metrics["vx_max_after_stop"] = after
            checks["params_restored"] = (
                after is not None
                and vx_standard is not None
                and abs(after - vx_standard) < 1e-6
            )
            checks["status_idle"] = self.status == "idle"
        return {"checks": checks, "metrics": metrics, "passed": all(checks.values())}

    def _save_csv(self, name, samples):
        os.makedirs(LOGS, exist_ok=True)
        suffix = f"_{self.tag}" if self.tag else ""
        with open(f"{LOGS}/follow_{name}{suffix}_poses.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                ["t", "rx", "ry", "ryaw", "px", "py", "gap", "status", "clearance"]
            )
            w.writerows(samples)

    def run(self):
        for _ in range(self.repeat):
            for name in self.names:
                try:
                    self.results.append(self.run_one(name))
                except Exception as e:  # one broken scenario must not end the suite
                    self.get_logger().error(f"[{name}] crashed: {e!r}")
                    self.results.append(
                        {"name": name, "passed": False, "error": repr(e)}
                    )
                self.save()
        passed = sum(bool(r.get("passed")) for r in self.results)
        self.get_logger().info(f"Follow test: {passed}/{len(self.results)} passed")
        for r in self.results:
            self.get_logger().info(
                f"  {r['name']}: {'PASS' if r.get('passed') else 'FAIL'} "
                f"{r.get('error', '')} {r.get('checks', '')} {r.get('metrics', '')}"
            )
        return passed == len(self.results)

    def save(self):
        os.makedirs(LOGS, exist_ok=True)
        suffix = f"_{self.tag}" if self.tag else ""
        with open(f"{LOGS}/follow_test_results{suffix}.json", "w") as f:
            json.dump({"results": self.results}, f, indent=1, default=_plain)


def main():
    rclpy.init()
    node = SimFollowTest()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.navigation.follow_person(False)
        node.track(False)
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
