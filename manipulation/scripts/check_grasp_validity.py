#!/usr/bin/env python3
"""Diagnóstico #944 §6.6 — ¿el grasp goal choca con el octomap contaminado?

Llama /check_state_validity (moveit_msgs/GetStateValidity) sobre un estado
articular del xarm6 e imprime CADA contacto: qué links chocan, contra qué
cuerpo (<octomap>, otro link, objeto), posición del contacto y profundidad.

Goal documentado como fallido (handoff §8.3):
  [-1.4162, 0.1166, -0.5465, 1.3619, 0.1085, -0.3174]   (joint1..joint6)

Uso (dentro del contenedor home2-manipulation, sourced):
  python3 check_grasp_validity.py
  python3 check_grasp_validity.py -- -1.4162 0.1166 -0.5465 1.3619 0.1085 -0.3174
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
DEFAULT_GOAL = [-1.4162, 0.1166, -0.5465, 1.3619, 0.1085, -0.3174]
# Dedos abiertos (envolvente más ancha = peor caso de auto-colisión).
# OJO gripper_assem (gripper-2026): joints en CamelCase. Un nombre inválido hace
# que move_group CRASHEE con SIGABRT (getVariableIndex lanza y MoveIt no lo captura).
FINGER_JOINTS = ["rightFinger", "leftFinger"]
FINGER_VALUES = [0.0, 0.0]

BODY_TYPE = {0: "ROBOT_LINK", 1: "WORLD_OBJECT", 2: "ROBOT_ATTACHED"}


def main():
    goal = DEFAULT_GOAL
    if "--" in sys.argv:
        vals = sys.argv[sys.argv.index("--") + 1:]
        if len(vals) == 6:
            goal = [float(v) for v in vals]

    rclpy.init()
    node = Node("check_grasp_validity")
    cli = node.create_client(GetStateValidity, "/check_state_validity")
    if not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("/check_state_validity no disponible")
        rclpy.shutdown()
        sys.exit(1)

    js = JointState()
    js.name = ARM_JOINTS + FINGER_JOINTS
    js.position = list(goal) + FINGER_VALUES

    rs = RobotState()
    rs.joint_state = js
    rs.is_diff = True  # merge sobre el estado/escena actual (conserva octomap vivo)

    req = GetStateValidity.Request()
    req.robot_state = rs
    req.group_name = "xarm6"

    print(f"\nGoal (joint1..6): {goal}")
    print("Grupo: xarm6 | dedos: abiertos (0.0)\n" + "-" * 70)

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    res = fut.result()
    if res is None:
        print("Sin respuesta del servicio (timeout).")
        rclpy.shutdown()
        sys.exit(2)

    print(f"VALID = {res.valid}")
    print(f"#contactos = {len(res.contacts)}\n")

    octo = []
    for c in res.contacts:
        b1, b2 = c.contact_body_1, c.contact_body_2
        t1 = BODY_TYPE.get(c.body_type_1, c.body_type_1)
        t2 = BODY_TYPE.get(c.body_type_2, c.body_type_2)
        p = c.position
        line = (f"  {b1}[{t1}] <-> {b2}[{t2}]  "
                f"pos=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f}) depth={c.depth:.4f}")
        print(line)
        if "octomap" in b1.lower() or "octomap" in b2.lower():
            octo.append((b1, b2, (p.x, p.y, p.z), c.depth))

    print("-" * 70)
    if octo:
        links = sorted({(b1 if "octomap" not in b1.lower() else b2) for b1, b2, _, _ in octo})
        print(f"⚠️  {len(octo)} contacto(s) con <octomap>. Links culpables: {links}")
        print("    => CONFIRMA hipótesis §6.6 (robot fantasma en el octomap).")
        print("    Posiciones de los voxels que bloquean (frame de planeación):")
        for b1, b2, pos, d in octo:
            link = b1 if "octomap" not in b1.lower() else b2
            print(f"      {link:16s} voxel@({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) depth={d:.4f}")
    else:
        print("Sin contactos contra <octomap>. El bloqueo NO es el octomap fantasma.")
        if res.contacts:
            print("    Revisar contactos link<->link (¿auto-colisión / SRDF?).")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
