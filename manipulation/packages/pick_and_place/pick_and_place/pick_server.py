#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.action import PickAction
from pick_and_place.utils.MoveItPlanner import MoveItPlanner


class PickActionServer(Node):
    def __init__(self):
        super().__init__("pick_server")
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            PickAction,
            "pick_action_server",
            self.execute_callback,
            callback_group=self.callback_group,
        )
        self.planner = MoveItPlanner(self, self.callback_group)
        self.planner.set_velocity(0.15)
        self.planner.set_acceleration(0.15)
        self.planner.set_planner("RRTConnect")
        self.get_logger().info("Pick Action Server has been started")

    async def execute_callback(self, goal_handle):
        """Execute the pick action when a goal is received."""
        self.get_logger().info("Executing pick goal...")

        # Initialize result
        feedback = PickAction.Feedback()
        result = PickAction.Result()
        try:
            result.success = self.pick(goal_handle, feedback)
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Pick failed: {str(e)}")
            goal_handle.abort()
            result.success = False
            return result

    def pick(self, goal_handle, feedback):
        """Perform the pick operation."""
        self.get_logger().info(
            f"Trying to pick up object: {goal_handle.request.object_name}"
        )
        grasping_poses = goal_handle.request.grasping_poses
        for i, pose in enumerate(grasping_poses):
            # Move to pre-grasp pose
            result = self.planner.plan_pose_goal(
                pose,
                wait=True,
            )
            print(f"Grasp Pose {i} result: {result}")
            if result:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(5)
    pick_server = PickActionServer()
    executor.add_node(pick_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
