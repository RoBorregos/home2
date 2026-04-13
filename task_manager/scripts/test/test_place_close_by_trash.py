#!/usr/bin/env python3

"""
Test script for place_close_by function with trash_can as target.
This script tests the place_close_by() function with "top" position
to place objects on top of the trash can.

Usage:
    ros2 run task_manager test_place_close_by_trash.py
"""

import rclpy
from rclpy.node import Node
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks


class TestPlaceCloseByTrash(Node):
    """Test node for place_close_by with trash_can target."""

    def __init__(self):
        super().__init__("test_place_close_by_trash")

        # Initialize ManipulationTasks manager
        self.manipulation_manager = ManipulationTasks(self, task=Task.DEBUG, mock_data=False)

        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("=" * 60)
        self.get_logger().info("Test Place Close By (Trash) started.")
        self.get_logger().info("=" * 60)

        # Run the test sequence
        self.run()

    def run(self):
        """Execute the test sequence for place_close_by on trash_can."""

        self.get_logger().info("\n[STEP 1] Starting place_close_by test with trash_can")
        self.get_logger().info("-" * 60)

        # Test parameters
        target_object = "trash_can"
        position = "top"
        max_distance = 0.30  # 30cm xy-plane (not used for "top", but included)
        max_height = 0.15  # 15cm z-direction safety margin

        self.get_logger().info(f"Target object: {target_object}")
        self.get_logger().info(f"Position: {position}")
        self.get_logger().info(f"Max distance (xy): {max_distance}m")
        self.get_logger().info(f"Max height (z): {max_height}m")
        self.get_logger().info("-" * 60)

        # Call place_close_by
        self.get_logger().info("\n[STEP 2] Sending place_close_by request...")
        result = self.manipulation_manager.place_close_by(
            target_object=target_object,
            position=position,
            max_distance=max_distance,
            max_height=max_height,
        )

        # Check result
        self.get_logger().info(f"\n[STEP 3] Result received: {result}")
        self.get_logger().info("-" * 60)

        if result == Status.EXECUTION_SUCCESS:
            self.get_logger().info("✓ SUCCESS: Object placed on top of trash_can!")
            self.get_logger().info("=" * 60)
        else:
            self.get_logger().error(f"✗ FAILED: Result status = {result}")
            self.get_logger().error("=" * 60)

        return result

    def test_multiple_positions(self):
        """
        Optional: Test multiple positions for place_close_by.
        Uncomment and call this method to test different positions.
        """
        test_cases = [
            {
                "target": "trash_can",
                "position": "top",
                "max_distance": 0.30,
                "max_height": 0.15,
                "description": "Place on top of trash_can",
            },
            {
                "target": "trash_can",
                "position": "close",
                "max_distance": 0.25,
                "max_height": 0.20,
                "description": "Place close to trash_can (default)",
            },
            {
                "target": "trash_can",
                "position": "front",
                "max_distance": 0.20,
                "max_height": 0.20,
                "description": "Place in front of trash_can",
            },
        ]

        for i, test in enumerate(test_cases, 1):
            self.get_logger().info(f"\n{'=' * 60}")
            self.get_logger().info(f"Test Case {i}: {test['description']}")
            self.get_logger().info(f"{'=' * 60}")

            result = self.manipulation_manager.place_close_by(
                target_object=test["target"],
                position=test["position"],
                max_distance=test["max_distance"],
                max_height=test["max_height"],
            )

            status_str = (
                "✓ SUCCESS" if result == Status.EXECUTION_SUCCESS else f"✗ FAILED ({result})"
            )
            self.get_logger().info(f"Result: {status_str}")


def main(args=None):
    """Main entry point for the test script."""
    rclpy.init(args=args)

    try:
        node = TestPlaceCloseByTrash()
        # Keep node alive for a moment after test
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
