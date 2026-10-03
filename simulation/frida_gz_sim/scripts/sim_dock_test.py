#!/usr/bin/env python3
"""Sim test for the planned base placement (table_docker + approach_planner).

Spawns test furniture into the arena, then for each scenario drives to a staging
pose with the real NavigationTasks API, docks / approaches with a SURFACE TYPE (no
hardcoded distance) and checks the result against the robot's Gazebo pose:

  * the gap between the footprint and the surface is inside its reach band and
    close to the preferred gap,
  * the robot is square to the surface (or facing the round table's centre),
  * it stands in front of the target object (lateral error),
  * the footprint does not overlap ANY obstacle (walls, chair, table).

Results go to /workspace/sim_logs/dock_test_results.json, with wall-clock event
times so recordings can be cut at the interesting moments.

    ros2 run frida_gz_sim sim_dock_test.py
    ros2 run frida_gz_sim sim_dock_test.py --ros-args -p scenarios:="island_chair,round_table_target"
"""

import json
import math
import os
import subprocess
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from frida_gz_sim import ground_truth as gt
from frida_gz_sim.nav import (
    MAP_NAME,
    ROUND_TABLE,
    TEST_CHAIR_SEAT,
    TEST_FURNITURE,
    TEST_TARGET,
)
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from frida_constants.navigation_constants import (
    GO_TO_POSE_SERVICE,
    MOVE_LOCATION_SERVICE,
)
from frida_interfaces.srv import GoToPose, MoveLocation
from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from visualization_msgs.msg import Marker, MarkerArray

WORLD = f"arena_{MAP_NAME}"
RESULTS = "/workspace/sim_logs/dock_test_results.json"


def _plain(o):
    """numpy scalars -> python for json."""
    return o.item() if hasattr(o, "item") else str(o)


# Staging is not what is tested and the sim can run well below real time, so it
# gets its own generous timeout instead of NavigationTasks' 90 s.
STAGING_TIMEOUT = 600.0

# Pass criteria
GAP_TOL = 0.03  # m from the profile's preferred gap
YAW_TOL = 4.0  # deg from square to the surface
LATERAL_TOL = 0.06  # m target off the robot's centreline
MIN_CLEARANCE = 0.0  # footprint must not overlap anything

# (location, sublocation) or ("pose", x, y, yaw_deg) staging; surface; target (map)
SCENARIOS = {
    "round_table_target": {
        "staging": ("kitchen", "dinner_table"),
        "surface": "round_table",
        "target": (-0.15, -13.95),
        "check": "disc",
    },
    "counter_lateral": {
        "staging": ("kitchen", "counter"),
        "surface": "counter",
        "target": (-2.95, -12.85),
        "check": "line",
        "region": (-3.2, -2.75, -13.9, -12.35),
    },
    "island_chair": {
        "staging": ("pose", -0.8, -8.0, -90.0),
        "surface": "table",
        "target": TEST_TARGET,
        "check": "line",
        "region": "island_table",
        "blocked": True,  # the chair stands right in front of the target
    },
    "dishwasher": {
        "staging": ("kitchen", "dishwasher"),
        "surface": "dishwasher",
        "target": None,
        "check": "line",
        "region": (-3.2, -2.75, -16.2, -15.0),  # wall only, not the jut above it
    },
    "cabinet": {
        "staging": ("kitchen", "cabinet"),
        "surface": "cabinet",
        "target": None,
        "check": "line",
        "region": (
            0.8,
            1.4,
            -11.75,
            -11.4,
        ),  # the cabinet front between its side panels
    },
    "legacy_offset": {
        # Old DockTable call (no surface type) at the dishwasher: offset 0.32 must
        # land on the same gap the new 'dishwasher' profile gives.
        "staging": ("kitchen", "dishwasher"),
        "legacy_offset": 0.32,
        "surface": "",
        "target": None,
        "check": "line",
        "region": (-3.2, -2.75, -16.2, -15.0),  # wall only, not the jut above it
    },
    "approach_point": {
        "staging": ("kitchen", "dishwasher"),
        "approach_point": (-2.55, -14.6),
        "check": "point",
    },
}


def gazebo_pose(model="frida", attempts=3):
    """Ground-truth (x, y, yaw); gz CLI calls can time out on a loaded machine."""
    for _ in range(attempts):
        try:
            out = subprocess.run(
                ["gz", "model", "-m", model, "-p"],
                capture_output=True,
                text=True,
                timeout=15,
            ).stdout
        except (subprocess.SubprocessError, FileNotFoundError):
            continue
        vals = [
            [float(v) for v in line.strip()[1:-1].split()]
            for line in out.splitlines()
            if line.strip().startswith("[") and line.strip().endswith("]")
        ]
        if len(vals) >= 2:
            return vals[0][0], vals[0][1], vals[1][2]
        time.sleep(1.0)
    return None


def box_sdf(name, x, y, yaw, sx, sy, h, z0=0.0, rgba="0.55 0.35 0.2 1", collide=False):
    geo = f"<geometry><box><size>{sx} {sy} {h}</size></box></geometry>"
    col = f"<collision name='c'>{geo}</collision>" if collide else ""
    return (
        f"<sdf version='1.9'><model name='{name}'><static>true</static>"
        f"<pose>{x} {y} {z0 + h / 2} 0 0 {yaw}</pose><link name='l'>{col}"
        f"<visual name='v'>{geo}<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
        f"</material></visual></link></model></sdf>"
    )


class SimDockTest(Node):
    def __init__(self):
        super().__init__("sim_dock_test")
        names = self.declare_parameter("scenarios", "").value
        self.names = [n for n in names.split(",") if n] if names else list(SCENARIOS)
        self.settle_time = self.declare_parameter("settle_time", 2.0).value
        self.navigation = NavigationTasks(self, task=Task.DEBUG)
        self.move_client = self.create_client(MoveLocation, MOVE_LOCATION_SERVICE)
        self.pose_client = self.create_client(GoToPose, GO_TO_POSE_SERVICE)
        share = get_package_share_directory("nav_main")
        with open(f"{share}/config/approach_profiles.yaml") as f:
            self.profiles = yaml.safe_load(f)
        maps = f"{get_package_share_directory('map_context')}/maps"
        with open(f"{maps}/{MAP_NAME}.yaml") as f:
            meta = yaml.safe_load(f)
        self.walls = gt.map_obstacle_points(
            f"{maps}/{meta['image']}",
            meta["resolution"],
            meta["origin"],
            meta["occupied_thresh"],
        )
        self.furniture = {
            n: gt.box_points(f["x"], f["y"], f["yaw"], f["sx"], f["sy"])
            for n, f in TEST_FURNITURE.items()
        }
        self.all_obstacles = np.vstack([self.walls, *self.furniture.values()])
        self.areas = self._areas()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/sim/furniture_markers", qos
        )
        self.create_timer(1.0, self._publish_furniture)
        self.events = []
        self.results = []

    # ------------------------------------------------------------- world
    def _areas(self):
        status, areas = self.navigation.retrieve_areas()
        if status == Status.EXECUTION_SUCCESS and isinstance(areas, dict):
            return areas
        path = f"{get_package_share_directory('map_context')}/maps/areas/areas_{MAP_NAME}.json"
        with open(path) as f:
            return json.load(f)

    def gz_service(self, service, reqtype, req):
        return subprocess.run(
            [
                "gz",
                "service",
                "-s",
                f"/world/{WORLD}/{service}",
                "--reqtype",
                reqtype,
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                req,
            ],
            capture_output=True,
            text=True,
            timeout=20,
        ).stdout

    def spawn_furniture(self):
        self.remove_furniture()
        for name, f in TEST_FURNITURE.items():
            sdf = box_sdf(
                f"test_{name}", f["x"], f["y"], f["yaw"], f["sx"], f["sy"], f["h"]
            )
            self.gz_service("create", "gz.msgs.EntityFactory", f'sdf: "{sdf}"')
        s = TEST_CHAIR_SEAT
        self.gz_service(
            "create",
            "gz.msgs.EntityFactory",
            f'sdf: "{box_sdf("test_chair_seat", s["x"], s["y"], 0, s["sx"], s["sy"], 0.04, s["z"])}"',
        )
        mug = box_sdf(
            "test_target_mug", *TEST_TARGET, 0, 0.08, 0.08, 0.1, 0.75, "0.9 0.5 0.0 1"
        )
        self.gz_service("create", "gz.msgs.EntityFactory", f'sdf: "{mug}"')
        self.get_logger().info(f"Spawned {len(TEST_FURNITURE) + 2} test models")

    def remove_furniture(self):
        for name in [*TEST_FURNITURE, "chair_seat", "target_mug"]:
            self.gz_service(
                "remove", "gz.msgs.Entity", f'name: "test_{name}" type: MODEL'
            )

    def _publish_furniture(self):
        arr = MarkerArray()
        for i, (name, f) in enumerate(TEST_FURNITURE.items()):
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "furniture"
            m.id = i
            m.type = Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (
                f["x"],
                f["y"],
                f["h"] / 2,
            )
            m.pose.orientation.z, m.pose.orientation.w = (
                math.sin(f["yaw"] / 2),
                math.cos(f["yaw"] / 2),
            )
            m.scale.x, m.scale.y, m.scale.z = f["sx"], f["sy"], f["h"]
            m.color.r, m.color.g, m.color.b, m.color.a = 0.55, 0.35, 0.2, 0.8
            arr.markers.append(m)
        seat = Marker()
        seat.header.frame_id = "map"
        seat.ns = "furniture"
        seat.id = 100
        seat.type = Marker.CUBE
        s = TEST_CHAIR_SEAT
        seat.pose.position.x, seat.pose.position.y, seat.pose.position.z = (
            s["x"],
            s["y"],
            s["z"],
        )
        seat.pose.orientation.w = 1.0
        seat.scale.x, seat.scale.y, seat.scale.z = s["sx"], s["sy"], 0.04
        seat.color.r, seat.color.g, seat.color.b, seat.color.a = 0.3, 0.3, 0.8, 0.35
        arr.markers.append(seat)
        self.marker_pub.publish(arr)

    # ----------------------------------------------------------- helpers
    def event(self, scenario, what):
        self.events.append({"t": time.time(), "scenario": scenario, "event": what})
        self.get_logger().info(f"[{scenario}] {what}")

    def wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def stage(self, staging):
        if staging[0] == "pose":
            _, x, y, yaw = staging
            req = GoToPose.Request()
            req.target_pose.header.frame_id = "map"
            req.target_pose.pose.position.x, req.target_pose.pose.position.y = x, y
            req.target_pose.pose.orientation.z = math.sin(math.radians(yaw) / 2)
            req.target_pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)
            client = self.pose_client
        else:
            req = MoveLocation.Request(location=staging[0], sublocation=staging[1])
            client = self.move_client
        if not client.wait_for_service(timeout_sec=10.0):
            return Status.EXECUTION_ERROR, "staging service unavailable"
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=STAGING_TIMEOUT)
        res = future.result()
        if res is None:
            return Status.EXECUTION_ERROR, "staging timed out"
        return (
            Status.EXECUTION_SUCCESS if res.success else Status.EXECUTION_ERROR
        ), res.error

    def perceived(self, target):
        """The target as perception would report it: in base_link, from the robot's
        TRUE pose (so localization error does not leak into the check, exactly like
        a camera detection on the real robot)."""
        if not target:
            return None
        pose = gazebo_pose()
        tb = gt.to_base(np.array([target]), *pose)[0]
        pt = PointStamped()
        pt.header.frame_id = "base_link"
        pt.point = Point(x=float(tb[0]), y=float(tb[1]))
        return pt

    def surface_points(self, sc):
        if sc["check"] == "disc":
            r = ROUND_TABLE
            near = (
                np.hypot(self.walls[:, 0] - r["x"], self.walls[:, 1] - r["y"])
                < r["r"] + 0.15
            )
            return self.walls[near]
        if sc["region"] in self.furniture:
            return self.furniture[sc["region"]]
        x0, x1, y0, y1 = sc["region"]
        m = (
            (self.walls[:, 0] >= x0)
            & (self.walls[:, 0] <= x1)
            & (self.walls[:, 1] >= y0)
            & (self.walls[:, 1] <= y1)
        )
        return self.walls[m]

    def profile_band(self, sc):
        if sc.get("legacy_offset"):
            gap = sc["legacy_offset"] + 0.19 - gt.FOOTPRINT[:, 0].max()
            return gap, gap - GAP_TOL, gap + GAP_TOL
        p = self.profiles["surfaces"][sc["surface"]]
        return p["gap"], p["gap_min"], p["gap_max"]

    # ------------------------------------------------------------ checks
    def evaluate(self, name, sc, pose):
        out = {"pose": [round(v, 3) for v in pose], "checks": {}}
        clear_all, nearest = gt.clearance(self.all_obstacles, pose, radius=2.5)
        out["min_clearance"] = round(clear_all, 3)
        out["checks"]["no_collision"] = clear_all > MIN_CLEARANCE
        x, y, yaw = pose
        if sc["check"] == "point":
            tx, ty = sc["approach_point"]
            prof = self.profiles["points"]["default"]
            d = math.hypot(tx - x, ty - y)
            face = math.degrees(
                abs(
                    (math.atan2(ty - y, tx - x) - yaw + math.pi) % (2 * math.pi)
                    - math.pi
                )
            )
            out.update(distance=round(d, 3), facing_err_deg=round(face, 1))
            out["checks"]["in_band"] = (
                prof["gap_min"] - 0.05 <= d <= prof["gap_max"] + 0.05
            )
            out["checks"]["facing"] = face <= 15.0
            return out
        gap, gmin, gmax = self.profile_band(sc)
        surf = self.surface_points(sc)
        sgap, _ = gt.clearance(surf, pose, radius=2.5)
        out.update(
            gap=round(sgap, 3),
            gap_preferred=round(gap, 3),
            band=[round(gmin, 3), round(gmax, 3)],
        )
        out["checks"]["gap_in_band"] = gmin - 0.01 <= sgap <= gmax + 0.01
        out["checks"]["gap_near_preferred"] = abs(sgap - gap) <= GAP_TOL
        if sc["check"] == "disc":
            r = ROUND_TABLE
            bearing = math.atan2(r["y"] - y, r["x"] - x)
            yaw_err = math.degrees(
                abs((bearing - yaw + math.pi) % (2 * math.pi) - math.pi)
            )
        else:
            # Square to the TRUE front edge (the arena walls are not axis-aligned and
            # are several cells thick): in the robot frame keep the nearest surface
            # point per 2 cm lateral strip in front of the robot, fit a line to that
            # edge and measure its tilt.
            b = gt.to_base(surf, x, y, yaw)
            b = b[(b[:, 0] > 0) & (np.abs(b[:, 1]) <= 0.45)]
            strips = np.round(b[:, 1] / 0.02).astype(int)
            edge = np.array(
                [
                    b[strips == k][np.argmin(b[strips == k][:, 0])]
                    for k in np.unique(strips)
                ]
            )
            if len(edge) < 3:
                out["checks"]["square"] = False
                out["error"] = "surface not in front of the robot"
                return out
            slope = np.polyfit(edge[:, 1], edge[:, 0], 1)[0]  # dx/dy of the edge
            yaw_err = math.degrees(abs(math.atan(slope)))
            # The arena is 5 cm cells: one cell of step over a short edge is already
            # a few degrees of measurement uncertainty, on top of the requirement.
            span = float(edge[:, 1].max() - edge[:, 1].min())
            out["yaw_tol_deg"] = round(
                YAW_TOL + math.degrees(math.atan(0.05 / max(span, 0.05))), 1
            )
        out["yaw_err_deg"] = round(yaw_err, 1)
        out["checks"]["square"] = yaw_err <= out.get("yaw_tol_deg", YAW_TOL)
        if sc.get("target"):
            tb = gt.to_base(np.array([sc["target"]]), x, y, yaw)[0]
            out["target_lateral"] = round(float(tb[1]), 3)
            # A blocked target may legitimately be off-centre: then the chosen pose
            # must still be collision-free; only unblocked targets must be centred.
            out["checks"]["in_front_of_target"] = abs(tb[1]) <= (
                LATERAL_TOL if not sc.get("blocked") else 0.8
            )
        return out

    # --------------------------------------------------------------- run
    def run_one(self, name):
        sc = SCENARIOS[name]
        self.event(name, f"staging {sc['staging']}")
        status, err = self.stage(sc["staging"])
        if status != Status.EXECUTION_SUCCESS:
            self.event(name, f"staging failed: {err}")
            return {"name": name, "passed": False, "error": f"staging: {err}"}
        self.wait(self.settle_time)
        self.event(name, "staged")
        start = time.time()
        if sc["check"] == "point":
            pt = PointStamped()
            pt.header.frame_id = "map"
            pt.point = Point(x=sc["approach_point"][0], y=sc["approach_point"][1])
            status, err = self.navigation.approach_point(pt)
        else:
            target = self.perceived(sc.get("target"))
            if sc.get("legacy_offset"):
                status, err = self.navigation.dock_table(offset=sc["legacy_offset"])
            else:
                status, err = self.navigation.dock_table(
                    surface_type=sc["surface"], target=target
                )
        elapsed = time.time() - start
        self.wait(self.settle_time)
        self.event(name, "done")
        pose = gazebo_pose()
        res = {
            "name": name,
            "service_ok": status == Status.EXECUTION_SUCCESS,
            "service_msg": err,
            "elapsed_s": round(elapsed, 1),
        }
        if pose is None:
            res.update(passed=False, error="no gazebo pose")
            return res
        res.update(self.evaluate(name, sc, pose))
        res["checks"] = {k: bool(v) for k, v in res["checks"].items()}
        res["passed"] = bool(res["service_ok"] and all(res["checks"].values()))
        self.get_logger().info(
            f"[{name}] {'PASS' if res['passed'] else 'FAIL'} {json.dumps(res, default=_plain)}"
        )
        return res

    def run(self):
        self.spawn_furniture()
        self.wait(2.0)
        for name in self.names:
            self.results.append(self.run_one(name))
            self.save()
        passed = sum(r["passed"] for r in self.results)
        self.get_logger().info(f"Dock test: {passed}/{len(self.results)} passed")
        for r in self.results:
            self.get_logger().info(
                f"  {r['name']}: {'PASS' if r['passed'] else 'FAIL'} "
                f"{ {k: v for k, v in r.items() if k not in ('name', 'passed')} }"
            )
        return passed == len(self.results)

    def save(self):
        os.makedirs(os.path.dirname(RESULTS), exist_ok=True)
        with open(RESULTS, "w") as f:
            json.dump(
                {"results": self.results, "events": self.events},
                f,
                indent=1,
                default=_plain,
            )


def main():
    rclpy.init()
    node = SimDockTest()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
