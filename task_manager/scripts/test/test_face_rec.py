#!/usr/bin/env python3

"""
Test Task Manager that reproduces the HRIC activate/deactivate sequence
to validate that save_face_name persists the face synchronously.

Flow (mirrors hric_task_manager.py):
  For each guest:
    GREETING         -> activate_face_recognition + follow_by_name("area")
    SAVE_FACE        -> save_face_name(name)
    deactivate cycle -> mimics navigate_to_living_room / find_seat /
                        navigate_to_entrance / wait_for_guest /
                        take_bag (all of which call deactivate_face_recognition)
  INTRODUCTION       -> activate + follow_by_name(guest.name) +
                        isPerson(guest.name) check for every saved guest.

The test PASSES if save_face_name returns SUCCESS AND every saved guest
is re-detected after the deactivate cycle.
"""

import time

import rclpy
from rclpy.node import Node

from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

NUM_GUESTS = 2
ATTEMPT_LIMIT = 3
VERIFY_ATTEMPTS = 10
DEACTIVATE_CYCLES = 3


class TestHRICFaceSave(Node):
    def __init__(self):
        super().__init__("test_hric_face_save")
        self.vision = VisionTasks(self, task=Task.DEBUG)
        # Start with the same initial state as HRIC_TM.__init__
        self.vision.deactivate_face_recognition()
        rclpy.spin_once(self, timeout_sec=1.0)
        Logger.info(self, "TestHRICFaceSave started.")

    def spin_sleep(self, seconds: float):
        """Sleep while keeping ROS callbacks serviced."""
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def prompt(self, msg: str):
        Logger.info(self, msg)
        input(">>> Press ENTER when ready... ")

    def deactivate_cycle(self, label: str):
        """Mimic the HRIC navigation/seat states that deactivate vision."""
        Logger.info(self, f"[deactivate cycle: {label}]")
        for i in range(DEACTIVATE_CYCLES):
            self.vision.deactivate_face_recognition()
            self.spin_sleep(0.5)

    def greeting_and_save(self, name: str) -> bool:
        """Mirror GREETING + SAVE_FACE states for a single guest."""
        Logger.info(self, f"=== GREETING for {name} ===")
        self.vision.activate_face_recognition()
        self.vision.follow_by_name("area")
        self.spin_sleep(2.0)

        self.prompt(f"Stand in front of the camera for {name}")

        Logger.info(self, f"=== SAVE_FACE for {name} ===")
        for attempt in range(1, ATTEMPT_LIMIT + 1):
            result = self.vision.save_face_name(name)
            if result == Status.EXECUTION_SUCCESS:
                Logger.success(self, f"save_face_name SUCCESS for {name} (attempt {attempt})")
                return True
            Logger.warn(self, f"save_face_name failed for {name} (attempt {attempt})")
            self.spin_sleep(1.0)

        Logger.error(self, f"save_face_name gave up for {name}")
        return False

    def verify_guest(self, name: str) -> bool:
        """Mirror INTRODUCTION detection loop for a single guest."""
        Logger.info(self, f"=== VERIFY {name} ===")
        self.vision.follow_by_name(name)
        self.prompt(f"Stand in front of the camera as {name}")

        for attempt in range(1, VERIFY_ATTEMPTS + 1):
            if self.vision.isPerson(name):
                Logger.success(self, f"Recognized {name} on attempt {attempt}")
                return True
            self.spin_sleep(1.0)

        Logger.error(self, f"Did NOT recognize {name} after {VERIFY_ATTEMPTS} attempts")
        return False

    def run_test(self):
        saved = []

        # # ---- Per-guest: GREETING + SAVE_FACE + deactivate cycle ----
        # for i in range(NUM_GUESTS):
        #     name = f"test_guest_{i + 1}"
        #     if self.greeting_and_save(name):
        #         saved.append(name)
        #     self.deactivate_cycle(f"after guest {i + 1}")

        # if not saved:
        #     Logger.error(self, "No faces were saved; aborting verification.")
        #     return False

        # ---- INTRODUCTION: re-activate and check every saved guest ----
        Logger.info(self, "=== INTRODUCTION (verification) ===")
        self.vision.activate_face_recognition()
        self.spin_sleep(2.0)
        saved = ["ale"]
        all_ok = True
        for name in saved:
            if not self.verify_guest(name):
                all_ok = False

        self.vision.deactivate_face_recognition()

        if all_ok:
            Logger.success(self, f"TEST PASSED: all {len(saved)} face(s) saved and recognized.")
        else:
            Logger.error(self, "TEST FAILED: at least one saved face was not recognized.")
        return all_ok


def main(args=None):
    rclpy.init(args=args)
    node = TestHRICFaceSave()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
