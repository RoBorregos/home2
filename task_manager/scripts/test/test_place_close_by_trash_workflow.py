#!/usr/bin/env python3

"""
Comprehensive test script for place_close_by with trash_can.
This script tests a complete workflow:
1. Move arm to scanning position
2. Pick an object (e.g., "pear")
3. Call TrashcanDetection service to automatically detect trash_can location
4. Place object on top of trash_can using place_close_by with detected coordinates
5. Return to initial position

Usage:
    ros2 run task_manager test_place_close_by_trash_workflow.py

Requirements:
    - manipulation_core running
    - vision detector nodes running
    - object_detection_handler running
    - trash_detection_node running
"""

import rclpy
from rclpy.node import Node
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from frida_interfaces.srv import TrashcanDetection
from frida_constants.vision_constants import TRASHCAN_SERVICE
import time


class TestPlaceCloseByTrashWorkflow(Node):
    """Test node for complete place_close_by workflow with trash_can."""

    def __init__(self):
        super().__init__("test_place_close_by_trash_workflow")

        # Initialize ManipulationTasks manager
        self.manipulation_manager = ManipulationTasks(self, task=Task.DEBUG, mock_data=False)

        # Create client for trash_can detection service
        self.trashcan_detection_client = self.create_client(TrashcanDetection, TRASHCAN_SERVICE)

    def run_workflow(self):
        """Execute the complete workflow."""

        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("COMPLETE WORKFLOW: Place Object on Top of Trash Can")
        self.get_logger().info("=" * 70)

        self.get_logger().info("\n[WORKFLOW] Starting placement workflow...")
        self.get_logger().info("-" * 70)

        # Step 3: Try to place WITHOUT explicit trash_can detection first
        # (in case trash_can is already detected in the system)
        self.get_logger().info(
            "\n[STEP 3/5] Attempting place_close_by (trying with already-detected trash_can)..."
        )
        self.get_logger().info("  - Target: trash_can")
        self.get_logger().info("  - Position: top")
        self.get_logger().info("  - Max distance: 0.30m (xy-plane)")
        self.get_logger().info("  - Max height: 0.15m (z-direction)")

        result = self.manipulation_manager.place_close_by(
            target_object="trash-can",
            position="top",
            max_distance=0.30,
            max_height=0.15,
            start_position="place_in_trash_pose",
            return_position="place_in_trash_pose",
        )

        if result == Status.EXECUTION_SUCCESS:
            self.get_logger().info(
                "✓ Successfully placed object on top of trash_can (using already-detected trash)"
            )
        else:
            # If place failed, now explicitly detect trash_can and retry
            self.get_logger().info(
                "\n[STEP 4/5] Place failed - Detecting trash_can location using vision service..."
            )
            trash_can_detected, trash_can_coords = self.detect_trash_can()

            if not trash_can_detected:
                self.get_logger().error("✗ Failed to detect trash_can")
                return Status.EXECUTION_ERROR

            self.get_logger().info(f"✓ Trash can detected at coordinates: {trash_can_coords}")

            # Retry place_close_by after trash detection
            self.get_logger().info(
                "\n[STEP 5/5] Retrying place_close_by with detected trash_can..."
            )
            result = self.manipulation_manager.place_close_by(
                target_object="trash_can",
                position="top",
                max_distance=0.30,
                max_height=0.15,
                start_position="place_in_trash_pose",
                return_position="place_in_trash_pose",
            )

            if result != Status.EXECUTION_SUCCESS:
                self.get_logger().error(f"✗ Failed to place on trash_can after detection: {result}")
                return result

            self.get_logger().info(
                "✓ Successfully placed object on top of trash_can (after explicit detection)"
            )

        # Success!
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("✓ COMPLETE WORKFLOW SUCCESS!")
        self.get_logger().info("  All steps completed successfully:")
        self.get_logger().info("  3. ✓ Placed on trash_can (top)")
        self.get_logger().info("     └─ Used already-detected trash, or detected + retried")
        self.get_logger().info("=" * 70 + "\n")

        return Status.EXECUTION_SUCCESS

    def detect_trash_can(self, max_retries: int = 3) -> tuple:
        """
        Call TrashcanDetection service to detect trash_can location.
        """
        self.get_logger().info(f"Attempting trash_can detection (max {max_retries} retries)...")

        for attempt in range(1, max_retries + 1):
            try:
                if not self.trashcan_detection_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warn(
                        f"  [{attempt}/{max_retries}] Service not available, retrying..."
                    )
                    time.sleep(1.0)
                    continue

                self.get_logger().info(
                    f"  [{attempt}/{max_retries}] Calling TrashcanDetection service..."
                )
                request = TrashcanDetection.Request()
                future = self.trashcan_detection_client.call_async(request)

                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if not future.done():
                    self.get_logger().warn(
                        f"  [{attempt}/{max_retries}] Service call timeout, retrying..."
                    )
                    continue

                result = future.result()

                if not result.success:
                    self.get_logger().warn(
                        f"  [{attempt}/{max_retries}] Service returned success=False"
                    )
                    time.sleep(1.0)
                    continue

                if not result.detection_array.detections:
                    self.get_logger().warn(
                        f"  [{attempt}/{max_retries}] No detections found, retrying..."
                    )
                    time.sleep(1.0)
                    continue

                detection = result.detection_array.detections[0]
                trash_can_coords = {
                    "x": detection.point3d.point.x,
                    "y": detection.point3d.point.y,
                    "z": detection.point3d.point.z,
                    "frame_id": detection.point3d.header.frame_id,
                    "label": detection.label_text,
                }

                self.get_logger().info(
                    f"  [{attempt}/{max_retries}] ✓ Trash can detected! "
                    f"Label: {detection.label_text}, "
                    f"Position: ({trash_can_coords['x']:.3f}, {trash_can_coords['y']:.3f}, {trash_can_coords['z']:.3f})"
                )

                return True, trash_can_coords

            except Exception as e:
                self.get_logger().error(
                    f"  [{attempt}/{max_retries}] Exception during detection: {e}"
                )
                if attempt < max_retries:
                    time.sleep(1.0)
                    continue

        self.get_logger().error(f"✗ Failed to detect trash_can after {max_retries} attempts")
        return False, None


def main(args=None):
    """Main entry point for the workflow test script."""
    rclpy.init(args=args)

    node = None
    try:
        node = TestPlaceCloseByTrashWorkflow()
        node.run_workflow()

    except KeyboardInterrupt:
        print("\nWorkflow interrupted by user.")
    except Exception as e:
        print(f"Error during workflow: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
