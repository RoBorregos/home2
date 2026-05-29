#!/usr/bin/env python3
"""¿El octomap de MoveIt tiene datos? (para descartar falso VALID por octomap vacío)."""
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents


def main():
    rclpy.init()
    n = Node("check_octomap")
    cli = n.create_client(GetPlanningScene, "/get_planning_scene")
    if not cli.wait_for_service(timeout_sec=10.0):
        print("/get_planning_scene no disponible"); rclpy.shutdown(); sys.exit(1)
    req = GetPlanningScene.Request()
    req.components = PlanningSceneComponents()
    req.components.components = PlanningSceneComponents.OCTOMAP
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    res = fut.result()
    if res is None:
        print("sin respuesta"); rclpy.shutdown(); sys.exit(2)
    oc = res.scene.world.octomap.octomap
    print(f"octomap: frame={res.scene.world.octomap.header.frame_id} "
          f"resolution={oc.resolution} binary={oc.binary} "
          f"data_bytes={len(oc.data)}")
    if len(oc.data) > 0:
        print(f"=> OCTOMAP POBLADO ({len(oc.data)} bytes). VALID=True es real.")
    else:
        print("=> OCTOMAP VACIO. Un VALID=True seria falso positivo.")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
