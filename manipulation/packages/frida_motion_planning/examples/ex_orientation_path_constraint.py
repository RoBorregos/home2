#!/usr/bin/env python3
"""
Example of moving to a joint configuration with orientation path constraints.
- ros2 run pymoveit2 ex_orientation_path_constraints.py --ros-args -p use_orientation_constraint:=True
- ros2 run pymoveit2 ex_orientation_path_constraints.py --ros-args -p use_orientation_constraint:=False
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from frida_interfaces.action import MoveToPose
from frida_interfaces.msg import Constraint
from geometry_msgs.msg import PoseStamped
import frida_motion_planning.robots.xarm6 as robot


class MoveToPoseContraintedClient(Node):
    def __init__(self):
        super().__init__("move_to_pose_client_with_constraint")
        self._action_client = ActionClient(
            self, MoveToPose, "/manipulation/move_to_pose_action_server"
        )
        # Publisher for the debug PoseStamped topic
        self._debug_pub = self.create_publisher(
            PoseStamped, "/call_pose_goal/debug_pose", 10
        )

    def publish_debug_pose(self, pose):
        self._debug_pub.publish(pose)
        self.get_logger().info("Published debug pose to /call_pose_goal/debug_pose")

    # Let the server pick the default values
    def send_goal(
        self,
        pose=PoseStamped(),
        velocity=0.2,
        acceleration=0.0,
        planner_id="",
        useConstraint=False,
    ):
        goal_msg = MoveToPose.Goal()
        goal_msg.pose = pose
        goal_msg.velocity = velocity
        goal_msg.acceleration = acceleration
        goal_msg.planner_id = planner_id
        goal_msg.apply_constraint = useConstraint

        if useConstraint:
            constraints_msg = Constraint()
            constraints_msg.orientation = pose.pose.orientation
            constraints_msg.frame_id = robot.base_link_name()
            constraints_msg.target_link = robot.end_effector_name()
            constraints_msg.tolerance_orientation = [3.14159, 0.5, 0.5]
            constraints_msg.weight = 0.5
            constraints_msg.parameterization = 1
            goal_msg.constraint = constraints_msg

        self._action_client.wait_for_server()
        self.get_logger().info("Sending pose goal with constraint...")

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveToPoseContraintedClient()

    # Define your pose goal here
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "link_base"  # Set appropriate frame
    pose_stamped.header.stamp = action_client.get_clock().now().to_msg()

    # Set the pose component
    pose_stamped.pose.position.x = 0.2
    pose_stamped.pose.position.y = 0.4
    pose_stamped.pose.position.z = 0.4
    # Quaternion for 90 degree roll (rotating around x-axis)
    pose_stamped.pose.orientation.x = 0.7071
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = -0.7071
    # Use Constraint
    apply_constraint = True    

    # Publish debug pose
    action_client.publish_debug_pose(pose_stamped)
    # Give some time for the message to be sent
    time.sleep(0.1)

    # Send goal
    future = action_client.send_goal(pose_stamped, useConstraint=apply_constraint)

    # Wait for goal to be accepted
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        action_client.get_logger().error("Goal rejected")
        rclpy.shutdown()
        return

    action_client.get_logger().info("Goal accepted")

    # Get result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(action_client, result_future)

    result = result_future.result().result
    if result.success:
        action_client.get_logger().info("Goal succeeded!")
    else:
        action_client.get_logger().error("Goal failed!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
