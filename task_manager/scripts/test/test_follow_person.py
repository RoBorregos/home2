#!/usr/bin/env python3

"""Interactive FOLLOW-PERSON test.

Starts the robot following a person and keeps following until you type 'stop'
(or 'q' / Ctrl-C). On exit it always stops the follow and the tracker, so the
base is left idle.

Flow (uses the same subtask managers the real tasks use):
    1. (optional) VisionTasks.track_person(True)  -> tracker locks the person in
       view and publishes /vision/tracking_results.
    2. NavigationTasks.follow_person(True)        -> nav_central sends a
       NavigateToPose goal with the follow BT and person_goal_smoother switches
       nav2 to follow-mode params, chasing the moving /goal_update.
    3. wait for you to say 'stop'.
    4. NavigationTasks.follow_person(False) + VisionTasks.track_person(False).

Prerequisites — bring up the nav + vision stack first, e.g.:
    ros2 launch nav_main hric.launch.py                 # omnibase by default
    ros2 launch vision_general hric_launch.py           # tracker_node

Usage:
    ros2 run task_manager test_follow_person.py
    # set the tracking target some other way (UI / another node) and only toggle nav:
    ros2 run task_manager test_follow_person.py --ros-args -p track:=false
    # offline dry-run (mocks every service call):
    ros2 run task_manager test_follow_person.py --ros-args -p mocked:=true
"""

import rclpy
from rclpy.node import Node

from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.utils.logger import Logger
from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks

STOP_WORDS = {"stop", "s", "q", "quit", "exit"}


def _ok(result) -> bool:
    """Normalize the two return conventions:
    NavigationTasks methods return (Status, message); VisionTasks return Status.
    """
    if isinstance(result, tuple):
        return len(result) > 0 and result[0] == Status.EXECUTION_SUCCESS
    return result == Status.EXECUTION_SUCCESS


class TestFollowPerson(Node):
    def __init__(self):
        super().__init__("test_follow_person")
        self.mocked = self.declare_parameter("mocked", False).value
        # When True, this test also drives the vision tracker (track_person).
        # Set False if you select the target elsewhere (e.g. the vision UI).
        self.track = self.declare_parameter("track", True).value

        print(f"\n{Logger.BOLD}=== Follow-Person interactive test ==={Logger.RESET}")
        print(f"  mocked={self.mocked}  track={self.track}")
        print("  -> type 'stop' (or 'q') + Enter to stop, or press Ctrl-C\n")

        self.nav = NavigationTasks(self, task=Task.DEBUG, mock_data=self.mocked)
        self.vision = (
            VisionTasks(self, task=Task.HELP_ME_CARRY, mock_data=self.mocked)
            if self.track
            else None
        )

        self.run()

    def run(self):
        following = False
        tracking = False
        try:
            # 1. Start the vision tracker on the person in view (optional)
            if self.vision is not None:
                print(f"{Logger.BOLD}-> Starting person tracking...{Logger.RESET}")
                res = self.vision.track_person(True)
                if not _ok(res):
                    print(
                        f"{Logger.RED}Could not start tracking ({res}). "
                        f"Make sure a person is in view and the tracker is running."
                        f"{Logger.RESET}"
                    )
                    return
                tracking = True

            # 2. Start nav follow
            print(f"{Logger.BOLD}-> Starting follow...{Logger.RESET}")
            res = self.nav.follow_person(True)
            if not _ok(res):
                print(f"{Logger.RED}Could not start follow ({res}).{Logger.RESET}")
                return
            following = True
            print(
                f"{Logger.GREEN}{Logger.BOLD}Following.{Logger.RESET} "
                f"The robot is now chasing the tracked person."
            )

            # 3. Block until the user says stop
            while True:
                try:
                    cmd = input("follow> ").strip().lower()
                except EOFError:
                    break
                if cmd in STOP_WORDS:
                    break
                if cmd:
                    print("   (type 'stop' to end)")
        except KeyboardInterrupt:
            print("\n(Ctrl-C received) stopping...")
        finally:
            # 4. Always stop follow + tracking, best-effort
            if following:
                print(f"{Logger.BOLD}-> Stopping follow...{Logger.RESET}")
                self.nav.follow_person(False)
            if tracking and self.vision is not None:
                print(f"{Logger.BOLD}-> Stopping tracking...{Logger.RESET}")
                self.vision.track_person(False)
            print(f"{Logger.GREEN}Done. Robot idle.{Logger.RESET}\n")


def main(args=None):
    rclpy.init(args=args)
    node = TestFollowPerson()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
