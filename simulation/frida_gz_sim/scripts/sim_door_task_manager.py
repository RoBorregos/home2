#!/usr/bin/env python3
"""Sim door task manager: drive to the door, open it with OPEN_DOOR, drive through it.

Uses the real NavigationTasks and the manipulation action, and scores from ground truth:
the door hinge angle and the robot's Gazebo pose on the far side of the wall.
"""

import math
import time

import rclpy
from frida_constants.manipulation_constants import MOVE_JOINTS_ACTION_SERVER
from frida_constants.xarm_configurations import NAV_POSE
from frida_interfaces.action import ManipulationAction, MoveJoints
from frida_interfaces.msg import ManipulationTask
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sim_nav_task_manager import gazebo_pose
from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.utils.status import Status
from task_manager.utils.task import Task

MANIPULATION_ACTION_SERVER = "/manipulation/manipulation_action_server"
# The door wall is the world plane x = 0; the robot starts at x > 0
THROUGH_X = -0.5
OPEN_ANGLE = 70.0  # deg the door must reach to count as open


class SimDoorTM(Node):
    def __init__(self):
        super().__init__("sim_door_task_manager")
        self.before = self.declare_parameter("before", "door/pre_door").value.split("/")
        self.after = self.declare_parameter("after", "door/beyond_door").value.split(
            "/"
        )
        self.navigation = NavigationTasks(self, task=Task.DEBUG)
        self.manipulation = ActionClient(
            self, ManipulationAction, MANIPULATION_ACTION_SERVER
        )
        self.move_joints = ActionClient(self, MoveJoints, MOVE_JOINTS_ACTION_SERVER)
        self.hinge = 0.0
        self.create_subscription(
            JointState,
            "/sim/door/joint_states",
            lambda msg: setattr(self, "hinge", max(self.hinge, msg.position[0])),
            10,
        )

    def wait_future(self, future, timeout: float):
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result() if future.done() else None

    def open_door(self) -> bool:
        if not self.manipulation.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Manipulation action server not available")
            return False
        goal = ManipulationAction.Goal()
        goal.task_type = ManipulationTask.OPEN_DOOR
        handle = self.wait_future(self.manipulation.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            self.get_logger().error("OPEN_DOOR goal rejected")
            return False
        result = self.wait_future(handle.get_result_async(), 300.0)
        return bool(result is not None and result.result.success)

    def tuck_arm(self) -> bool:
        """Arm to the nav pose before driving through."""
        if not self.move_joints.wait_for_server(timeout_sec=10.0):
            return False
        goal = MoveJoints.Goal()
        goal.joint_names = list(NAV_POSE["joints"])
        goal.joint_positions = [math.radians(v) for v in NAV_POSE["joints"].values()]
        goal.velocity = 0.3
        handle = self.wait_future(self.move_joints.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return False
        result = self.wait_future(handle.get_result_async(), 60.0)
        return bool(result is not None and result.result.success)

    def drive(self, location: str, sublocation: str) -> bool:
        status, error = self.navigation.move_to_location(location, sublocation)
        ok = status == Status.EXECUTION_SUCCESS
        self.get_logger().info(
            f"{location}/{sublocation}: {'reached' if ok else f'failed: {error}'}"
        )
        return ok

    def run(self):
        steps = [
            ("drive to the door", lambda: self.drive(*self.before)),
            ("open the door", self.open_door),
            ("tuck the arm", self.tuck_arm),
            ("drive through", lambda: self.drive(*self.after)),
        ]
        for name, step in steps:
            start = time.time()
            ok = step()
            self.get_logger().info(
                f"{name}: {'ok' if ok else 'FAILED'} in {time.time() - start:.0f}s"
            )
            if not ok:
                break
        pose = gazebo_pose()
        through = pose is not None and pose[0] < THROUGH_X
        opened = math.degrees(self.hinge) >= OPEN_ANGLE
        self.get_logger().info("===== SIM DOOR REPORT =====")
        self.get_logger().info(
            f"  door opened to {math.degrees(self.hinge):.0f} deg: {'ok' if opened else 'no'}"
        )
        self.get_logger().info(
            f"  robot through the door: {'ok' if through else 'no'} (pose {pose})"
        )
        self.get_logger().info(
            f"  RESULT {'success' if opened and through else 'failure'}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimDoorTM()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
