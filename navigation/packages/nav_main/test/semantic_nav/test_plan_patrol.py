#!/usr/bin/env python3
"""PlanPatrol against the live node: modes, filters, markers.

See README.md for how to run it."""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from frida_interfaces.srv import PlanPatrol

PASS = FAIL = 0


def check(name, ok, detail=""):
    global PASS, FAIL
    global_marker = "PASS" if ok else "FAIL"
    if ok:
        PASS += 1
    else:
        FAIL += 1
    print(f"  {global_marker} {name}" + (f" — {detail}" if detail else ""), flush=True)


class Driver(Node):
    def __init__(self):
        super().__init__("semantic_nav_driver")
        self.patrol = self.create_client(PlanPatrol, "/navigation/plan_patrol")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = None
        self.staleness = None
        self.create_subscription(MarkerArray, "/navigation/patrol/markers",
                                 lambda m: setattr(self, "markers", m), latched)
        self.create_subscription(String, "/navigation/patrol/staleness",
                                 lambda m: setattr(self, "staleness", m), latched)

    def call(self, mode="full", areas=(), max_viewpoints=0, timeout=20.0):
        request = PlanPatrol.Request()
        request.mode = mode
        request.areas = list(areas)
        request.max_viewpoints = max_viewpoints
        future = self.patrol.call_async(request)
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result() if future.done() else None

    def pump(self, seconds):
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Driver()
    print("\nesperando a /navigation/plan_patrol ...", flush=True)
    ready = node.patrol.wait_for_service(timeout_sec=60.0)
    check("el servicio PlanPatrol aparece", ready)
    if not ready:
        return 1
    node.pump(6.0)  # let the node fetch areas and the costmap

    res = node.call("full")
    check("PlanPatrol full responde", res is not None and res.success,
          res.error if res else "sin respuesta")
    if not res or not res.success:
        return 1
    n = len(res.viewpoints)
    check("devuelve los 16 muebles etiquetados", n == 16, f"{n} viewpoints")
    check("cada viewpoint trae área, mueble, pose de brazo y dwell",
          len(res.areas_out) == n and len(res.sublocations) == n
          and len(res.arm_poses) == n and len(res.dwell_s) == n)
    check("las poses van en el frame map",
          all(p.header.frame_id == "map" for p in res.viewpoints))
    check("la distancia total es razonable", 10.0 < res.total_distance < 60.0,
          f"{res.total_distance:.1f} m")
    check("los cuartos salen agrupados",
          sum(1 for a, b in zip(res.areas_out, res.areas_out[1:]) if a != b)
          == len(set(res.areas_out)) - 1,
          f"{len(set(res.areas_out))} cuartos")
    poses_ok = all(abs(p.pose.orientation.z) <= 1.0 and p.pose.orientation.w != 0.0
                   for p in res.viewpoints)
    check("las orientaciones son cuaterniones válidos", poses_ok)
    print("    ruta:", " -> ".join(f"{a}/{s}" for a, s in
                                   list(zip(res.areas_out, res.sublocations))[:5]), "...",
          flush=True)
    arm = {p for p in res.arm_poses}
    check("las poses de brazo son las nombradas del xarm",
          arm <= {"table_stare", "front_stare", "flat_stare", "look_side_stare",
                  "scan_floor_carry_bag_pose"}, str(sorted(arm)))

    quick = node.call("quick")
    check("PlanPatrol quick da una superficie por cuarto",
          quick is not None and quick.success and len(quick.viewpoints) == 4,
          f"{len(quick.viewpoints) if quick else 0} viewpoints")

    limited = node.call("full", max_viewpoints=5)
    check("max_viewpoints recorta", limited is not None and len(limited.viewpoints) == 5,
          f"{len(limited.viewpoints) if limited else 0}")
    check("la distancia se recalcula al recortar",
          limited is not None and limited.total_distance < res.total_distance,
          f"{limited.total_distance:.1f} m < {res.total_distance:.1f} m")

    one_room = node.call("full", areas=["kitchen"])
    check("filtrar por área funciona",
          one_room is not None and one_room.success
          and set(one_room.areas_out) == {"kitchen"},
          f"{len(one_room.viewpoints) if one_room else 0} viewpoints en kitchen")

    empty = node.call("full", areas=["cuarto_que_no_existe"])
    check("un área inexistente falla con mensaje, no con crash",
          empty is not None and not empty.success and bool(empty.error),
          empty.error if empty else "")

    revisit = node.call("revisit")
    check("PlanPatrol revisit responde",
          revisit is not None and revisit.success and len(revisit.viewpoints) == 16)

    node.pump(4.0)
    check("publica markers de RViz", node.markers is not None and len(node.markers.markers) > 1,
          f"{len(node.markers.markers) if node.markers else 0} markers")
    if node.markers:
        namespaces = {m.ns for m in node.markers.markers}
        check("markers con flechas y etiquetas", {"viewpoints", "labels"} <= namespaces,
              str(sorted(namespaces)))

    print(f"\n{PASS} pasaron, {FAIL} fallaron", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
