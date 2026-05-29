#!/usr/bin/env python3
"""FK del estado-goal para confirmar que los voxeles culpables coinciden con
la posición física de los links (auto-contaminación vs obstáculo real)."""
import sys, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GOAL = [-1.4162, 0.1166, -0.5465, 1.3619, 0.1085, -0.3174]
LINKS = ["gripper", "left_finger", "link6"]
# voxeles reportados por /check_state_validity
VOXELS = {"gripper": (0.384, -0.101, 0.886),
          "left_finger": (0.446, 0.059, 0.879),
          "link6": (0.356, -0.026, 0.924)}


def main():
    rclpy.init()
    n = Node("fk_check")
    cli = n.create_client(GetPositionFK, "/compute_fk")
    if not cli.wait_for_service(timeout_sec=10.0):
        print("/compute_fk no disponible"); rclpy.shutdown(); sys.exit(1)
    js = JointState(); js.name = ARM; js.position = list(GOAL)
    rs = RobotState(); rs.joint_state = js; rs.is_diff = True
    req = GetPositionFK.Request()
    req.header.frame_id = "base_link"
    req.fk_link_names = LINKS
    req.robot_state = rs
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    res = fut.result()
    print(f"\nFK frame: {req.header.frame_id}  err={res.error_code.val}\n" + "-"*64)
    for name, ps in zip(res.fk_link_names, res.pose_stamped):
        p = ps.pose.position
        vx = VOXELS.get(name)
        d = math.sqrt((p.x-vx[0])**2 + (p.y-vx[1])**2 + (p.z-vx[2])**2) if vx else float("nan")
        print(f"  {name:12s} origen=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f}) "
              f"voxel=({vx[0]:+.3f},{vx[1]:+.3f},{vx[2]:+.3f}) dist={d*100:.1f}cm")
    print("-"*64)
    print("dist link-origin <-> voxel pequeña (~radio del link) => voxel SOBRE el link")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
