#!/usr/bin/env python3

"""Standalone test for the wall_aligner precision node.

align_washing_machine -> (optional) close_washing_machine to an exact
perpendicular lidar distance, staying square on the live fit the whole drive.

Usage (integration container, robot parked roughly facing the machine):
    python3 test_wall_align.py                # align only, print calibration distance
    python3 test_wall_align.py 0.30           # align, then close to 0.30 m
    python3 test_wall_align.py 0.30 --center  # also center on the panel midpoint

Calibration: park the robot at the PERFECT final depth, run with no args and
read the reported distance — that number is the close target.
"""

import sys

import rclpy
from rclpy.node import Node

from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task


class TestWallAlign(Node):
    def __init__(self):
        super().__init__("test_wall_align")
        self.nav = NavigationTasks(self, task=Task.DEBUG)
        rclpy.spin_once(self, timeout_sec=1.0)

    def run(self, target: float, center: bool) -> int:
        Logger.info(self, f"Aligning to the machine (center={center})...")
        status, payload = self.nav.align_washing_machine(center=center)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"Align failed: {payload}")
            return 1
        Logger.success(
            self,
            f"Aligned: distance={payload['distance']:.3f} m "
            f"yaw_error={payload['yaw_error']:+.4f} rad "
            f"face={payload['segment_length']:.2f} m",
        )

        if target <= 0.0:
            Logger.info(
                self,
                f"No close target given — calibration distance is "
                f"{payload['distance']:.3f} m.",
            )
            return 0

        Logger.info(self, f"Closing to {target:.3f} m...")
        status, payload = self.nav.close_washing_machine(distance=target)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"Close failed: {payload}")
            return 2
        Logger.success(
            self,
            f"Closed: final_distance={payload['final_distance']:.3f} m "
            f"(traveled {payload['traveled']:+.3f} m)",
        )
        return 0


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    center = "--center" in sys.argv
    target = float(args[0]) if args else 0.0

    rclpy.init()
    node = TestWallAlign()
    try:
        rc = node.run(target, center)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
