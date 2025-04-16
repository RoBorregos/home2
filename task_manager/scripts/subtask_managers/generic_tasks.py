#!/usr/bin/env python3

from utils.subtask_manager import SubtaskManager


class GenericTask:
    """Class to manage the generic tasks"""

    def __init__(self, subtask_manager: SubtaskManager):
        """Initialize the class"""

        self.subtask_manager = subtask_manager
        # Initialize the subtask managers

    def test_function(self):
        """Test function"""
        self.logger.info("Test function")
        return True

    def talk(self):
        i = 0
        while i < 5:
            print("Hello, I am a generic task manager.")
            self.subtask_manager.hri.say("Hello, I am a generic task manager.")
            i += 1
            import time

            time.sleep(1)
        return True
