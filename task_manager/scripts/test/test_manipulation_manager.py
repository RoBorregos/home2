#!/usr/bin/env python3

"""
Task Manager for testing the subtask managers
"""

import rclpy

# from config.hri.debug import config as test_hri_config
from rclpy.node import Node
from utils.logger import Logger
import time
# from subtask_managers.hri_tasks import HRITasks

from subtask_managers.manipulation_tasks import ManipulationTasks

PICK_OBJECT = "zucaritas"
POUR_OBJECT = "blue_cereal"
CONTAINER = "cup"


TEST = "PICK"
# TEST = "PLACE"
# TEST = "NAV_POSE"
TEST = "PAN_TO"
# # TEST = "FOLLOW_FACE"
# TEST = "POUR"


class TestTaskManager(Node):
    def __init__(self):
        super().__init__("test_task_manager")
        self.subtask_manager = {}
        # self.subtask_manager["hri"] = HRITasks(self, config=test_hri_config)

        self.subtask_manager["manipulation"] = ManipulationTasks(self, task="DEMO", mock_data=False)

        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("TestTaskManager has started.")
        self.run()

    def run(self):
        # """testing vision tasks"""
        if TEST == "PICK":
            self.get_logger().info(f"Trying to pick {PICK_OBJECT}")
            result = self.subtask_manager["manipulation"].pick_object(PICK_OBJECT)
            self.get_logger().info(f"Result: {result}")
        elif TEST == "PLACE":
            self.get_logger().info(f"Trying to place {PICK_OBJECT}")
            result = self.subtask_manager["manipulation"].place()
            self.get_logger().info(f"Result: {result}")
        elif TEST == "PAN_TO":
            self.get_logger().info("Moving to front stare")
            self.subtask_manager["manipulation"].move_joint_positions(
                named_position="table_stare", velocity=0.3, degrees=True
            )
            joint_positions = self.subtask_manager["manipulation"].get_joint_positions(degrees=True)
            Logger.info(self, f"Joint positions: {joint_positions}")
            # joint_positions["joint1"] = joint_positions["joint1"] - 45.0
            # self.subtask_manager["manipulation"].move_joint_positions(
            #     joint_positions=joint_positions, velocity=0.5, degrees=True
            # )
            # Logger.info(self, "Moving back to original position")
            # joint_positions["joint1"] = joint_positions["joint1"] + 45.0
            # self.subtask_manager["manipulation"].move_joint_positions(
            #     joint_positions=joint_positions, velocity=0.5, degrees=True
            # )

        elif TEST == "NAV_POSE":
            self.get_logger().info("Moving to navigation pose")
            self.subtask_manager["manipulation"].move_joint_positions(
                named_position="nav_pose", velocity=0.3, degrees=True
            )
        elif TEST == "FOLLOW_FACE":
            self.get_logger().info("Moving to front stare")
            self.subtask_manager["manipulation"].move_joint_positions(
                named_position="front_stare", velocity=0.3, degrees=True
            )
            self.get_logger().info("Trying to follow face")
            result = self.subtask_manager["manipulation"].follow_face(True)
            self.get_logger().info(f"Result: {result}")
            self.get_logger().info("Waiting 10s until stopping follow face")
            self.get_logger().info("Stopping follow face")
            result = self.subtask_manager["manipulation"].follow_face(False)
            self.get_logger().info(f"Result: {result}")
            self.get_logger().info("Moving to front stare")
            self.subtask_manager["manipulation"].move_joint_positions(
                named_position="front_stare", velocity=0.3, degrees=True
            )
            self.get_logger().info("DONE")

            joint_positions = self.subtask_manager["manipulation"].get_joint_positions(degrees=True)
            Logger.info(self, f"Joint positions: {joint_positions}")
            joint_positions["joint1"] = joint_positions["joint1"] - 45.0
            self.subtask_manager["manipulation"].move_joint_positions(
                joint_positions=joint_positions, velocity=0.5, degrees=True
            )
            Logger.info(self, "Moving back to original position")
            joint_positions["joint1"] = joint_positions["joint1"] + 45.0
            self.subtask_manager["manipulation"].move_joint_positions(
                joint_positions=joint_positions, velocity=0.5, degrees=True
            )

        elif TEST == "POUR":
            self.subtask_manager["manipulation"].move_joint_positions(
                named_position="table_stare", velocity=0.5, degrees=True
            )
            time.sleep(2.5)
            for i in range(3):
                self.subtask_manager["manipulation"].pour(POUR_OBJECT, CONTAINER)


def main(args=None):
    rclpy.init(args=args)
    node = TestTaskManager()

    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
