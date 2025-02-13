#!/usr/bin/env python3

"""
Task Manager for Demos
"""

import rclpy
from rclpy.node import Node
# from config.hri.debug import config as test_hri_config

from subtask_managers.vision_tasks import VisionTasks
# from subtask_managers.hri_tasks import HRITasks

from utils.logger import Logger


class ManipulacionMock:
    def move_to(self, x, y):
        pass


class subtask_manager:
    vision: VisionTasks
    hri: HRITasks
    manipulation: ManipulacionMock


class DemoTaskManager(Node):
    """Class to manage demo tasks"""

    TASK_STATES = {"START": 0, "INTRODUCTION": 1, "RECEIVE_COMMAND": 2, "FOLLOW_FACE": 3}

    Multiplier = 5

    def __init__(self):
        """Initialize the node"""
        super().__init__("demo_task_manager")
        self.subtask_manager = subtask_manager()
        self.subtask_manager.vision = VisionTasks(self, task="DEMO", mock_data=False)
        self.subtask_manager.hri = HRITasks(self, config=test_hri_config)

        # change
        self.subtask_manager.manipulation = ManipulacionMock()

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

        self.x_delta_multiplier = self.Multiplier
        self.y_delta_multiplier = self.Multiplier / 2

        self.current_state = DemoTaskManager.TASK_STATES["FOLLOW_FACE"]

        self.get_logger().info("DemoTaskManager has started.")
        self.create_timer(0.1, self.run)

    def run(self):
        """Running main loop"""

        if self.current_state == DemoTaskManager.TASK_STATES["START"]:
            Logger.state(self, "Starting task")
            self.subtask_manager.hri.say(
                "Hi, I'm FRIDA, a service robot designed by RoBorregos. I can do several requests, just say my name to chat."
            )
            self.current_state = DemoTaskManager.TASK_STATES["FOLLOW_FACE"]

        if self.current_state == DemoTaskManager.TASK_STATES["INTRODUCTION"]:
            Logger.state(self, "Introduction task")
            # Wait until keyword is said
            self.subtask_manager.hri.say("Hi, I'm FRIDA, what is your name?",wait=True)
            text = self.subtask_manager.hri.hear()
            name = self.subtask_manager.hri.extract_data("name", text)
            self.subtask_manager.vision.save_face_name(name)
            self.subtask_manager.hri.say(f"Hello {name}, how can i help you?",wait=True)
            self.current_state = DemoTaskManager.TASK_STATES["RECEIVE_COMMAND"]

        if self.current_state == DemoTaskManager.TASK_STATES["RECEIVE_COMMAND"]:
            # Do sth to receive and parse basic commands (go to, pick, place)
            Logger.state(self, "Receive command task")
            text = self.subtask_manager.hri.hear()
            response = self.subtask_manager.hri.ask(text)
            self.subtask_manager.hri.say(response)

        if self.current_state == DemoTaskManager.TASK_STATES["FOLLOW_FACE"]:
            # Follow face task
            Logger.state(self, "Follow face task")
            if self.subtask_manager.vision.follow_face is None:
                continue
            x, y = self.subtask_manager.vision.follow_face
            new_x = min(
                max(x * self.x_delta_multiplier + self.current_x, self.min_x), self.max_x
            )

            new_y = min(
                max(y * self.y_delta_multiplier + self.current_y, self.min_y), self.max_y
            )

            if new_x != self.current_x or new_y != self.current_y:
                self.subtask_manager.manipulation.move_to(new_x, new_y)

            if self.subtask_manager.hri.keyword() = "frida":
                self.current_state = DemoTaskManager.TASK_STATES["INTRODUCTION"]



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
