#!/usr/bin/env python3

from utils.subtask_manager import SubtaskManager


class GenericTask:
    """Class to manage the generic tasks"""

    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1,
        "TARGET_NOT_FOUND": 2,
    }

    SUBTASKS = {
        "RECEPTIONIST": [],
        "RESTAURANT": [],
        "SERVE_BREAKFAST": [],
        "STORING_GROCERIES": [],
        "STICKLER_RULES": [],
        "DEMO": [],
    }

    def __init__(self, SubtaskManager: SubtaskManager):
        """Initialize the class"""

        self.subtask = SubtaskManager
        # Initialize the subtask managers

    def test_function(self):
        """Test function"""
        self.logger.info("Test function")
        return True

    def talk(self):
        i = 0
        while i < 5:
            print("Hello, I am a generic task manager.")
            self.subtask.hri.say("Hello, I am a generic task manager.")
            i += 1
            import time

            time.sleep(1)
        return True
