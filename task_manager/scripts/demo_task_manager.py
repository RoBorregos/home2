#!/usr/bin/env python3

"""
Task Manager for Demos
"""

import rclpy
from rclpy.node import Node
from config.hri.debug import config as test_hri_config

from subtask_managers.vision_tasks import VisionTasks
from subtask_managers.hri_tasks import HRITasks

from utils.logger import Logger


class DemoTaskManager(Node):
    """Class to manage demo tasks"""

    TASK_STATES = {"START": 0, "INTRODUCTION": 1, "RECEIVE_COMMAND": 2}

    def __init__(self):
        """Initialize the node"""
        super().__init__("demo_task_manager")
        self.subtask_manager = {}

        self.subtask_manager["vision"] = VisionTasks(self, taks="DEMO", mock_data=False)
        self.subtask_manager["hri"] = HRITasks(self, config=test_hri_config)

        self.current_state = DemoTaskManager.TASK_STATES["START"]

        self.get_logger().info("DemoTaskManager has started.")
        self.run()

    def run(self):
        """Running main loop"""

        while rclpy.ok():
            if self.current_state == DemoTaskManager.TASK_STATES["START"]:
                Logger.state(self, "Starting task")
                self.subtask_manager["hri"].say(
                    "Hi, I'm FRIDA, a service robot designed by RoBorregos. I can do several requests, just say my name to chat."
                )
                self.current_state = DemoTaskManager.TASK_STATES["INTRODUCTION"]

            if self.current_state == DemoTaskManager.TASK_STATES["INTRODUCTION"]:
                Logger.state(self, "Introduction task")
                # Wait until keyword is said
                self.subtask_manager["hri"].say("Hi, I'm FRIDA, what is your name?")
                name = self.subtask_manager["hri"].hear()
                self.subtask_manager["vision"].save_face_name(name)
                self.subtask_manager["hri"].say(f"Hello {name}, how can i help you?")
                self.current_state = DemoTaskManager.TASK_STATES["RECEIVE_COMMAND"]

            if self.current_state == DemoTaskManager.TASK_STATES["RECEIVE_COMMAND"]:
                # Do sth to receive and parse basic commands (go to, pick, place)
                Logger.state(self, "Receive command task")

            if self.current_state == DemoTaskManager.TASK_STATES["FOLLOW_FACE"]:
                # Follow face task
                Logger.state(self, "Follow face task")
                x, y = self.subtask_manager["vision"].follow_face()
