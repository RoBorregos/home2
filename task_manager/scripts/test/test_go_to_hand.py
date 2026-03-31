#!/usr/bin/env python3

"""
Test script for GoToHand - isolates the hand detection + arm movement pipeline.
Detects a hand, logs all coordinates and frame info, then attempts GoToHand.
Run: ros2 run task_manager test_go_to_hand
"""

import rclpy
from rclpy.node import Node
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks

ATTEMPT_LIMIT = 5


class TestGoToHand(Node):
    def __init__(self):
        super().__init__("test_go_to_hand")
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG, mock_data=False)
        self.vision = VisionTasks(self, task=Task.HRIC, mock_data=False)

        rclpy.spin_once(self, timeout_sec=2.0)
        self.get_logger().info("=== GoToHand Test Started ===")
        self.run()

    def run(self):
        # Step 1: Move to carry_pose so arm is in a good starting position
        self.get_logger().info("Step 1: Moving to carry_pose...")
        self.manipulation.move_to_position("carry_pose")

        # Step 2: Open gripper
        self.get_logger().info("Step 2: Opening gripper...")
        self.manipulation.open_gripper()

        self.get_logger().info("Please extend your hand towards the robot.")
        self.get_logger().info("Waiting 3 seconds before detection...")
        import time

        time.sleep(3)

        # Step 3: Detect hand
        for attempt in range(ATTEMPT_LIMIT):
            self.get_logger().info(
                f"Step 3: Detecting hand (attempt {attempt + 1}/{ATTEMPT_LIMIT})..."
            )
            status, hand_point = self.vision.detect_hand()

            if status != Status.EXECUTION_SUCCESS or hand_point is None:
                self.get_logger().warn(f"Hand detection failed: status={status}")
                time.sleep(2)
                continue

            # Log detailed info about the detected point
            self.get_logger().info("=== HAND POINT DETAILS ===")
            self.get_logger().info(f"  frame_id: '{hand_point.header.frame_id}'")
            self.get_logger().info(
                f"  stamp: {hand_point.header.stamp.sec}.{hand_point.header.stamp.nanosec}"
            )
            self.get_logger().info(f"  x: {hand_point.point.x:.4f}")
            self.get_logger().info(f"  y: {hand_point.point.y:.4f}")
            self.get_logger().info(f"  z: {hand_point.point.z:.4f}")
            self.get_logger().info("==========================")

            # Step 4: Attempt GoToHand
            self.get_logger().info("Step 4: Sending GoToHand...")
            go_result = self.manipulation.go_to_hand(
                point=hand_point,
                hand_offset=0.2,
            )

            if go_result == Status.EXECUTION_SUCCESS:
                self.get_logger().info("SUCCESS: GoToHand reached target!")
                # Step 5: Close gripper
                self.get_logger().info("Step 5: Closing gripper...")
                self.manipulation.close_gripper()
                time.sleep(2)

                # Step 6: Move back to carry pose
                self.get_logger().info("Step 6: Moving to carry_pose...")
                self.manipulation.move_to_position("carry_pose")
                return
            else:
                self.get_logger().error(f"GoToHand FAILED on attempt {attempt + 1}")
                time.sleep(2)

        self.get_logger().error("All GoToHand attempts failed!")
        self.get_logger().info("Moving back to nav_pose...")
        self.manipulation.move_to_position("nav_pose")


def main(args=None):
    rclpy.init(args=args)
    node = TestGoToHand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
