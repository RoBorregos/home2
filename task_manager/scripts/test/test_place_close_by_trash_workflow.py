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
        self.manipulation_manager = ManipulationTasks(
            self, task=Task.DEBUG, mock_data=False
        )
        
        # Create client for trash_can detection service
        self.trashcan_detection_client = self.create_client(
            TrashcanDetection, TRASHCAN_SERVICE
        )
        
        # NOTA: Se eliminó el código bloqueante de este __init__
        # La inicialización pesada y ejecución ahora viven en el bloque main()

    def run_workflow(self):
        """Execute the complete workflow."""
        
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("COMPLETE WORKFLOW: Place Object on Top of Trash Can")
        self.get_logger().info("=" * 70)

        self.get_logger().info("\n[WORKFLOW] Starting placement workflow...")
        self.get_logger().info("-" * 70)
        
        # Step 1: Move to table stare position (initial scanning position)
        self.get_logger().info("\n[STEP 1/5] Moving to table_stare position for object detection...")
        result = self.manipulation_manager.move_to_position("table_stare", velocity=0.3)
        if result != Status.EXECUTION_SUCCESS:
            self.get_logger().error(f"✗ Failed to move to table_stare: {result}")
            return result
        self.get_logger().info("✓ Successfully moved to table_stare")
        
        # Step 2: Pick an object
        object_to_pick = "pear"
        self.get_logger().info(f"\n[STEP 2/5] Picking object: '{object_to_pick}'...")
        result = self.manipulation_manager.pick_object(object_to_pick)
        if result != Status.EXECUTION_SUCCESS:
            self.get_logger().error(f"✗ Failed to pick object: {result}")
            return result
        self.get_logger().info(f"✓ Successfully picked '{object_to_pick}'")
        
        # Step 3: Detect trash_can using TrashcanDetection service
        self.get_logger().info("\n[STEP 3/5] Detecting trash_can location using vision service...")
        trash_can_detected, trash_can_coords = self.detect_trash_can()
        if not trash_can_detected:
            self.get_logger().error("✗ Failed to detect trash_can")
            return Status.EXECUTION_ERROR
        self.get_logger().info(f"✓ Trash can detected at coordinates: {trash_can_coords}")
        
        # Step 4: Place object on top of trash_can using place_close_by
        self.get_logger().info("\n[STEP 4/5] Placing object on top of trash_can...")
        self.get_logger().info("  - Target: trash_can")
        self.get_logger().info("  - Position: top")
        self.get_logger().info("  - Max distance: 0.30m (xy-plane)")
        self.get_logger().info("  - Max height: 0.15m (z-direction)")
        
        # ⚠️ ATENCIÓN: Si tu API requiere que le pases las coordenadas detectadas, agrégalas como parámetro abajo.
        # Ejemplo: target_pose=trash_can_coords o target_coordinates=trash_can_coords
        # Si manipulation_manager internamente lee el tópico TF o de detecciones, déjalo así.
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
        
        # Step 5: Return to initial position
        self.get_logger().info("\n[STEP 5/5] Returning to table_stare position...")
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
        self.get_logger().info("  3. ✓ Detected trash_can")
        self.get_logger().info("  4. ✓ Placed on trash_can (top)")
        self.get_logger().info("  5. ✓ Returned to table_stare")
        self.get_logger().info("=" * 70 + "\n")
        
        return Status.EXECUTION_SUCCESS

    def detect_trash_can(self, max_retries: int = 3) -> tuple:
        """
        Call TrashcanDetection service to detect trash_can location.
        
        Args:
            max_retries: Maximum number of detection attempts
            
        Returns:
            Tuple of (detected: bool, coordinates: dict or None)
            coordinates format: {'x': float, 'y': float, 'z': float}
        """
        self.get_logger().info(f"Attempting trash_can detection (max {max_retries} retries)...")
        
        for attempt in range(1, max_retries + 1):
            try:
                # Wait for service to be available
                if not self.trashcan_detection_client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warn(
                        f"  [{attempt}/{max_retries}] Service not available, retrying..."
                    )
                    time.sleep(1.0)
                    continue
                
                # Call the service
                self.get_logger().info(f"  [{attempt}/{max_retries}] Calling TrashcanDetection service...")
                request = TrashcanDetection.Request()
                future = self.trashcan_detection_client.call_async(request)
                
                # CORRECCIÓN: Usar el método nativo de ROS 2 para evitar deadlocks
                # Esto mantiene el nodo "vivo" (haciendo spin) hasta que el servicio responda o pase el timeout
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if not future.done():
                    self.get_logger().warn(
                        f"  [{attempt}/{max_retries}] Service call timeout, retrying..."
                    )
                    continue
                
                result = future.result()
                
                # Check if service returned successfully
                if not result.success:
                    self.get_logger().warn(
                        f"  [{attempt}/{max_retries}] Service returned success=False"
                    )
                    time.sleep(1.0)
                    continue
                
                # Check if detections were found
                if not result.detection_array.detections:
                    self.get_logger().warn(
                        f"  [{attempt}/{max_retries}] No detections found, retrying..."
                    )
                    time.sleep(1.0)
                    continue
                
                # Extract coordinates from first detection
                detection = result.detection_array.detections[0]
                trash_can_coords = {
                    'x': detection.point3d.point.x,
                    'y': detection.point3d.point.y,
                    'z': detection.point3d.point.z,
                    'frame_id': detection.point3d.header.frame_id,
                    'label': detection.label_text
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
        
        self.get_logger().error(
            f"✗ Failed to detect trash_can after {max_retries} attempts"
        )
        return False, None

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
    
    node = None
    try:
        node = TestPlaceCloseByTrashWorkflow()
        
        # CORRECCIÓN: Esperar a que el servicio esté listo antes de arrancar la misión
        node.get_logger().info("\n[INIT] Waiting for trash_detection_node service...")
        while rclpy.ok() and not node.trashcan_detection_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().warn(f"  → Service '{TRASHCAN_SERVICE}' not available, retrying...")
        node.get_logger().info(f"✓ Service '{TRASHCAN_SERVICE}' is now available!\n")
        
        # CORRECCIÓN: Ejecutar el workflow AQUI, de forma secuencial y limpia
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