#!/usr/bin/env python3
"""Second scenario: staleness marking while the robot moves, and costmap relocation."""
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from frida_interfaces.srv import PlanPatrol

# Paths are derived from this file, so the suite runs from a source checkout and
# from inside the test container without editing anything.
REPO = os.environ.get("REPO_ROOT") or os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..")
)
NAV_MAIN = os.path.join(REPO, "navigation", "packages", "nav_main")
MAPS = os.path.join(REPO, "navigation", "packages", "map_context", "maps")
if NAV_MAIN not in sys.path:
    sys.path.insert(0, NAV_MAIN)


from nav_main.semantic.surfaces import load_viewpoints  # noqa: E402

PASS = FAIL = 0


def check(name, ok, detail=""):
    global PASS, FAIL
    if ok:
        PASS += 1
    else:
        FAIL += 1
    print(f"  {'PASS' if ok else 'FAIL'} {name}" + (f" — {detail}" if detail else ""), flush=True)


class Driver(Node):
    def __init__(self):
        super().__init__("staleness_driver")
        self.patrol = self.create_client(PlanPatrol, "/navigation/plan_patrol")
        self.set_params = self.create_client(SetParameters, "/fake_nav/set_parameters")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.staleness = None
        self.create_subscription(String, "/navigation/patrol/staleness",
                                 lambda m: setattr(self, "staleness", m), latched)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/global_costmap/costmap", latched)

    def pump(self, seconds):
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_robot(self, x, y, yaw):
        request = SetParameters.Request()
        for name, value in (("robot_x", x), ("robot_y", y), ("robot_yaw", yaw)):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            request.parameters.append(p)
        future = self.set_params.call_async(request)
        deadline = time.time() + 5.0
        while rclpy.ok() and not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def call(self, mode="full", timeout=20.0):
        request = PlanPatrol.Request()
        request.mode = mode
        future = self.patrol.call_async(request)
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result() if future.done() else None

    def ages(self):
        if self.staleness is None:
            return {}
        payload = json.loads(self.staleness.data)
        return {f"{v['area']}/{v['sublocation']}": v["age_s"] for v in payload["viewpoints"]}


def main():
    rclpy.init()
    node = Driver()
    node.patrol.wait_for_service(timeout_sec=60.0)
    node.set_params.wait_for_service(timeout_sec=30.0)
    node.pump(3.0)

    areas = json.load(open(os.path.join(MAPS, "areas", "areas_robocup2026_1.json")))
    vps = {v.key: v for v in load_viewpoints(areas)}
    target = vps["kitchen/sink"]

    print("\n-- staleness: el robot se para frente al sink --", flush=True)
    check("se movió el robot al viewpoint", node.move_robot(target.x, target.y, target.yaw))
    node.pump(4.0)   # el timer de escaneo corre a 1 Hz

    ages = node.ages()
    check("se publicó el topic de staleness", bool(ages), f"{len(ages)} viewpoints")
    check("el sink quedó marcado como recién escaneado",
          ages.get("kitchen/sink") is not None and ages["kitchen/sink"] < 10.0,
          f"age={ages.get('kitchen/sink')}")
    otros = [k for k, v in ages.items() if v is not None and k != "kitchen/sink"]
    check("solo se marcó una superficie", not otros, f"también marcados: {otros}")

    revisit = node.call("revisit")
    check("revisit responde", revisit is not None and revisit.success)
    if revisit and revisit.success:
        orden = [f"{a}/{s}" for a, s in zip(revisit.areas_out, revisit.sublocations)]
        check("lo recién escaneado queda al final de revisit",
              orden[-1] == "kitchen/sink", f"último = {orden[-1]}")

    print("\n-- snapshot en disco --", flush=True)
    node.pump(12.0)  # el timer de snapshot corre cada 10 s
    path = "/workspace/log/semantic_nav/staleness_default.json"
    exists = os.path.exists(path)
    check("se escribió el snapshot", exists, path)
    if exists:
        snap = json.load(open(path))
        check("el snapshot trae la superficie escaneada",
              "kitchen/sink" in snap.get("ages", {}), str(list(snap.get("ages", {}))))
        check("el snapshot guarda el reloj de pared", "saved_at_wall" in snap)

    print("\n-- costmap: bloquear un viewpoint y ver si se reubica --", flush=True)
    blocked = vps["living_room/sofa"]
    base = node.call("full")
    original = None
    if base:
        for pose, area, sub in zip(base.viewpoints, base.areas_out, base.sublocations):
            if f"{area}/{sub}" == "living_room/sofa":
                original = (pose.pose.position.x, pose.pose.position.y)
    check("el sofá está en la ruta original", original is not None, str(original))

    # Publica un costmap con un cuadro ocupado encima del sofá.
    from nav_main.semantic.viewpoints import grid_from_pgm
    meta = {}
    for line in open(os.path.join(MAPS, "robocup2026_1.yaml")):
        line = line.split("#", 1)[0].strip()
        if ":" in line:
            k, v = (p.strip() for p in line.split(":", 1))
            meta[k] = json.loads(v) if v.startswith("[") else (float(v) if v.replace(".","",1).replace("-","",1).isdigit() else v)
    grid = grid_from_pgm(os.path.join(MAPS, "robocup2026_1.pgm"), meta)
    data = list(grid.data)
    reach = int(0.45 / grid.resolution)
    col0 = int((blocked.x - grid.origin_x) / grid.resolution)
    row0 = int((blocked.y - grid.origin_y) / grid.resolution)
    for drow in range(-reach, reach + 1):
        for dcol in range(-reach, reach + 1):
            c, r = col0 + dcol, row0 + drow
            if 0 <= c < grid.width and 0 <= r < grid.height:
                data[r * grid.width + c] = 100
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.info.resolution = grid.resolution
    msg.info.width = grid.width
    msg.info.height = grid.height
    msg.info.origin.position.x = grid.origin_x
    msg.info.origin.position.y = grid.origin_y
    msg.info.origin.orientation.w = 1.0
    msg.data = data
    for _ in range(3):
        node.costmap_pub.publish(msg)
        node.pump(0.5)

    after = node.call("full")
    moved = None
    if after:
        for pose, area, sub in zip(after.viewpoints, after.areas_out, after.sublocations):
            if f"{area}/{sub}" == "living_room/sofa":
                moved = (pose.pose.position.x, pose.pose.position.y)
    check("el sofá sigue en la ruta (no se descartó)", moved is not None, str(moved))
    if moved and original:
        shift = math.hypot(moved[0] - original[0], moved[1] - original[1])
        check("el viewpoint bloqueado se reubicó", shift > 0.05, f"se movió {shift:.2f} m")
        check("la reubicación es pequeña", shift < 2.0, f"{shift:.2f} m")
    check("la ruta conserva los 16 viewpoints", after is not None and len(after.viewpoints) == 16,
          f"{len(after.viewpoints) if after else 0}")

    print(f"\n{PASS} pasaron, {FAIL} fallaron", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
