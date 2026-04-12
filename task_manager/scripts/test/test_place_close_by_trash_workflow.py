#!/usr/bin/env python3

"""
Comprehensive test script for place_close_by with trash_can.
This script tests a complete workflow:
1. Move arm to scanning position
2. Pick an object (e.g., "banana")
3. Place object on top of trash_can using place_close_by
4. Return to initial position

Usage:
    ros2 run task_manager test_place_close_by_trash_workflow.py
"""

import rclpy
from rclpy.node import Node
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks


class TestPlaceCloseByTrashWorkflow(Node):
    """Test node for complete place_close_by workflow with trash_can."""

    def __init__(self):
        super().__init__("test_place_close_by_trash_workflow")

        # Initialize ManipulationTasks manager
        self.manipulation_manager = ManipulationTasks(
            self, task=Task.DEBUG, mock_data=False
        )

        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("COMPLETE WORKFLOW: Place Object on Top of Trash Can")
        self.get_logger().info("=" * 70)
        
        # Run the complete workflow
        self.workflow_status = self.run_workflow()

    def run_workflow(self):
        """Execute the complete workflow."""
        
        self.get_logger().info("\n[WORKFLOW] Starting placement workflow...")
        self.get_logger().info("-" * 70)
        
        # Step 1: Move to table stare position (initial scanning position)
        self.get_logger().info("\n[STEP 1/4] Moving to table_stare position for object detection...")
        result = self.manipulation_manager.move_to_position("table_stare", velocity=0.3)
        if result != Status.EXECUTION_SUCCESS:
            self.get_logger().error(f"✗ Failed to move to table_stare: {result}")
            return result
        self.get_logger().info("✓ Successfully moved to table_stare")
        
        # Step 2: Pick an object
        object_to_pick = "banana"
        self.get_logger().info(f"\n[STEP 2/4] Picking object: '{object_to_pick}'...")
        result = self.manipulation_manager.pick_object(object_to_pick)
        if result != Status.EXECUTION_SUCCESS:
            self.get_logger().error(f"✗ Failed to pick object: {result}")
            return result
        self.get_logger().info(f"✓ Successfully picked '{object_to_pick}'")
        
        # Step 3: Place object on top of trash_can using place_close_by
        self.get_logger().info("\n[STEP 3/4] Placing object on top of trash_can...")
        self.get_logger().info("  - Target: trash_can")
        self.get_logger().info("  - Position: top")
        self.get_logger().info("  - Max distance: 0.30m (xy-plane)")
        self.get_logger().info("  - Max height: 0.15m (z-direction)")
        
        result = self.manipulation_manager.place_close_by(
            target_object="trash_can",
            position="top",
            max_distance=0.30,
            max_height=0.15
        )
        
        if result != Status.EXECUTION_SUCCESS:
            self.get_logger().error(f"✗ Failed to place on trash_can: {result}")
            return result
        self.get_logger().info("✓ Successfully placed object on top of trash_can")
        
        # Step 4: Return to initial position
        self.get_logger().info("\n[STEP 4/4] Returning to table_stare position...")
        result = self.manipulation_manager.move_to_position("table_stare", velocity=0.3)
        if result != Status.EXECUTION_SUCCESS:
            self.get_logger().error(f"✗ Failed to return to table_stare: {result}")
            return result
        self.get_logger().info("✓ Successfully returned to table_stare")
        
        # Success!
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("✓ COMPLETE WORKFLOW SUCCESS!")
        self.get_logger().info("  All steps completed successfully:")
        self.get_logger().info("  1. ✓ Moved to table_stare")
        self.get_logger().info("  2. ✓ Picked object")
        self.get_logger().info("  3. ✓ Placed on trash_can (top)")
        self.get_logger().info("  4. ✓ Returned to table_stare")
        self.get_logger().info("=" * 70 + "\n")
        
        return Status.EXECUTION_SUCCESS

    def run_workflow_with_multiple_objects(self):
        """
        Optional: Test placing multiple objects on trash_can.
        Uncomment and call this method to test multiple object pickups.
        """
        objects_to_test = [
            "banana",
            "apple",
            "carrot"
        ]
        
        results = []
        
        for obj in objects_to_test:
            self.get_logger().info(f"\n{'=' * 70}")
            self.get_logger().info(f"Testing with object: {obj}")
            self.get_logger().info(f"{'=' * 70}")
            
            # Move to scanning position
            result = self.manipulation_manager.move_to_position("table_stare", velocity=0.3)
            if result != Status.EXECUTION_SUCCESS:
                self.get_logger().error(f"✗ Failed to move to scanning position")
                results.append((obj, Status.EXECUTION_ERROR))
                continue
            
            # Pick object
            result = self.manipulation_manager.pick_object(obj)
            if result != Status.EXECUTION_SUCCESS:
                self.get_logger().error(f"✗ Failed to pick {obj}")
                results.append((obj, Status.EXECUTION_ERROR))
                continue
            
            # Place on trash
            result = self.manipulation_manager.place_close_by(
                target_object="trash_can",
                position="top",
                max_distance=0.30,
                max_height=0.15
            )
            
            if result == Status.EXECUTION_SUCCESS:
                self.get_logger().info(f"✓ Successfully placed {obj} on trash_can")
                results.append((obj, Status.EXECUTION_SUCCESS))
            else:
                self.get_logger().error(f"✗ Failed to place {obj} on trash_can")
                results.append((obj, Status.EXECUTION_ERROR))
        
        # Summary
        self.get_logger().info(f"\n{'=' * 70}")
        self.get_logger().info("SUMMARY - Multiple Objects Test")
        self.get_logger().info(f"{'=' * 70}")
        for obj, status in results:
            status_str = "✓ SUCCESS" if status == Status.EXECUTION_SUCCESS else "✗ FAILED"
            self.get_logger().info(f"{obj}: {status_str}")
        self.get_logger().info(f"{'=' * 70}\n")


def main(args=None):
    """Main entry point for the workflow test script."""
    rclpy.init(args=args)
    
    try:
        node = TestPlaceCloseByTrashWorkflow()
        # Keep node alive
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        print("\nWorkflow interrupted by user.")
    except Exception as e:
        print(f"Error during workflow: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
