#!/usr/bin/env python3

"""
Task Manager for Demo follow face
"""

import rclpy
from rclpy.node import Node
from subtask_managers.manipulation_tasks import ManipulationTasks
from subtask_managers.vision_tasks import VisionTasks
from utils.logger import Logger


class subtask_manager:
    vision: VisionTasks
    manipulation: ManipulationTasks


class DemoTaskManager(Node):
    """Class to manage demo tasks"""

    TASK_STATES = {"START": 0, "FOLLOW_FACE": 3}

    Multiplier = 5

    def __init__(self):
        """Initialize the node"""
        super().__init__("demo_task_manager")
        self.subtask_manager = subtask_manager()
        self.subtask_manager.vision = VisionTasks(self, task="DEMO", mock_data=False)
        self.subtask_manager.manipulation = ManipulationTasks(self, task="DEMO", mock_data=False)
        # self.subtask_manager.hri = HRITasks(self, config=test_hri_config)

        # change

        self.current_x = 0
        self.current_y = 0
        self.max_x = 20
        self.min_x = -20
        self.max_y = 10
        self.min_y = -10
        self.max_delta_x = 2
        self.max_delta_y = 1
        self.min_delta_x = -2
        self.min_delta_y = -1
        self.prev = None

        self.x_delta_multiplier = self.Multiplier
        self.y_delta_multiplier = self.Multiplier / 2

        self.subtask_manager.manipulation.activate_arm()

        self.current_state = DemoTaskManager.TASK_STATES["FOLLOW_FACE"]

        self.get_logger().info("DemoTaskManager has started.")
        self.create_timer(0.1, self.run)

    def run(self):
        """Running main loop"""

        if self.current_state == DemoTaskManager.TASK_STATES["START"]:
            Logger.state(self, "Starting task")
            self.current_state = DemoTaskManager.TASK_STATES["FOLLOW_FACE"]

        if self.current_state == DemoTaskManager.TASK_STATES["FOLLOW_FACE"]:
            # Follow face task
            Logger.state(self, "Follow face task")
            x,y, asdasd = self.subtask_manager.vision.get_follow_face()
            if asdasd == self.prev: 
                pass
            else:
                self.prev = asdasd
            if (x is None):
                self.subtask_manager.manipulation.move_to(0.0, 0.0)
                pass
            else:
                
                print(f"x and y {x} {y}")
                if x > 0.09 or x < -0.09:
                    self.subtask_manager.manipulation.move_to(x, y)
                else:
                    self.subtask_manager.manipulation.move_to(0.0, 0.0)


def main(args=None):
    """Main function"""

    rclpy.init(args=args)

    demo_task_manager = DemoTaskManager()

    demo_task_manager.get_logger().info("DemoTaskManager has started.")
    # demo_task_manager.run()

    rclpy.spin(demo_task_manager)

    demo_task_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
