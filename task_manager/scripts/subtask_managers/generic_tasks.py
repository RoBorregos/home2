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
            self.subtask.hri.say(self.subtask.hri.node.test_text)
            i += 1
            import time

            time.sleep(1)
        return True

    # def follow_person(self):
    #     KP = 0.5
    #     while True:
    #         break
    #         # dashgo_cmd_vel = self.subtask.nav.cmd_vel
    #         # dashgo_rotation_vel_sign = np.sign(dashgo_cmd_vel.angular.z)
    #         # person_boinding_box_center = self.vision.detect_person()

    #         # base_rotational_velocitiy_sign = np.sign(cmd_vel_subscriptor_value)
    #         # arm_rotational_velocity = person_bounding_box_center_x * KP
    #         # arm_rotation_velocity_sign = np.sign(arm_rotational_velocity)

    #         # person_pose = self.subtask.vision.detect_person()
    #         # person_camera_point = self.subtask.vision.get_person_camera_point()
    #         # self.subtask.nav.follow_person(True,person_pose)
    #         # self.subtask.manipulation.pan_to()
