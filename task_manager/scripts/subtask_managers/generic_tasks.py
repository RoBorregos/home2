#!/usr/bin/env python3

from utils.subtask_manager import SubtaskManager


class GenericTask:
    """Class to manage the generic tasks"""

    def __init__(self, subtask_manager: SubtaskManager):
        """Initialize the class"""

        self.subtask_manager = subtask_manager

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

    def pan_to_person(self):
        """Pan to the person"""
        self.subtask_manager.hri.say("Please say stop when you want me to stop")
        KP = 0.5
        MAX_ERROR = 0.2
        while True:
            # This function should return the x coordinate of the person, -1 being
            # the leftmost and 1 being the rightmost, and 0 the center
            person_center_bounding_box = self.subtask_manager.vision.person_bounding_box
            # This function should return the current cmd_vel topic value
            # This function will convert the person_center_bounding_box to a
            # joint_velocity value
            joint_velocity = person_center_bounding_box * KP
            if abs(person_center_bounding_box) < MAX_ERROR:
                joint_velocity = 0
                self.subtask_manager.manipulation._send_joint_velocity(joint_velocity)
            else:
                self.subtask_manager.manipulation._send_joint_velocity(joint_velocity)
            _, result = self.subtask_manager.hri.hear()
            if "stop" in result.lower():
                self.subtask_manager.nav.follow_person(False)
                break

        # while True:

        #     # This function should return the x coordinate of the person, -1 being
        #     # the leftmost and 1 being the rightmost, and 0 the center
        #     person_center_bounding_box = self.subtask.vision.person_bounding_box
        #     # This function should return the current cmd_vel topic value
        #     dashgo_cmd_vel_sign = np.sign(self.subtask.nav.cmd_vel)

        #     # This function will convert the person_center_bounding_box to a
        #     # joint_velocitry value
        #     joint_velocity = person_center_bounding_box * KP
        #     self.subtask.manipulation._send_joint_velocity(joint_velocity)

        #     return True
