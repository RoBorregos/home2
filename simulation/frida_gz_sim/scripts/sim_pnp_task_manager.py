#!/usr/bin/env python3
"""Sim pick-and-place task manager: clear the dining table onto the side table.

Uses the real vision and manipulation subtask managers; navigation and HRI are not
involved because the robot base is static in front of the table.
"""

import time

import rclpy
import tf2_geometry_msgs  # noqa: F401 (registers PointStamped transforms)
from frida_constants.manipulation_constants import GRIPPER_GRASP_STATE_TOPIC
from frida_constants.vision_constants import CAMERA_FRAME
from frida_interfaces.msg import GripperGraspState
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from tf2_ros import Buffer, TransformException, TransformListener

# COCO labels the sim objects can be detected as
PICKABLE = {"bottle", "cup", "wine glass", "vase"}

# Dining table region in base_link (m); detections elsewhere (e.g. the side table) are ignored
TABLE_X = (0.30, 1.15)
TABLE_Y = (-0.60, 0.60)
TABLE_MIN_Z = 0.70

# Object in the middle of the side table that placements are aimed next to
PLACE_ANCHOR = "bowl"

# table_stare rotated to face the side table on the robot's left (degrees)
SIDE_TABLE_STARE = {
    "joint1": 0.0,
    "joint2": -80.0,
    "joint3": -70.0,
    "joint4": 0.0,
    "joint5": 50.0,
    "joint6": 45.0,
}


class SimPickAndPlaceTM(Node):
    def __init__(self):
        super().__init__("sim_pnp_task_manager")
        self.attempt_limit = self.declare_parameter("attempt_limit", 3).value
        self.max_objects = self.declare_parameter("max_objects", 10).value
        self.settle_time = self.declare_parameter("settle_time", 3.0).value
        self.vision = VisionTasks(self, task=Task.PICK_AND_PLACE)
        self.manipulation = ManipulationTasks(self, task=Task.PICK_AND_PLACE)
        self.attempts: dict[str, int] = {}
        self.results: list[tuple[str, str]] = []
        self.holding = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            GripperGraspState,
            GRIPPER_GRASP_STATE_TOPIC,
            lambda msg: setattr(self, "holding", msg.object_detected),
            10,
        )

    def wait(self, seconds: float):
        """Sleep while keeping the node spinning."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def on_dining_table(self, detection) -> bool:
        """True when the detection's 3D point lies over the dining table."""
        point = PointStamped()
        point.header.frame_id = CAMERA_FRAME
        point.point.x = float(detection.px)
        point.point.y = float(detection.py)
        point.point.z = float(detection.pz)
        try:
            p = self.tf_buffer.transform(
                point, "base_link", timeout=Duration(seconds=1.0)
            ).point
        except TransformException as e:
            self.get_logger().warn(f"Could not locate {detection.classname}: {e}")
            return False
        return (
            TABLE_X[0] < p.x < TABLE_X[1]
            and TABLE_Y[0] < p.y < TABLE_Y[1]
            and p.z > TABLE_MIN_Z
        )

    def go_to_table_stare(self) -> bool:
        """Move to table_stare, clearing the octomap when a failed motion left the arm in contact."""
        for _ in range(3):
            if (
                self.manipulation.move_to_position("table_stare")
                == Status.EXECUTION_SUCCESS
            ):
                return True
            self.get_logger().warn(
                "Could not reach table_stare; clearing the octomap and retrying"
            )
            self.manipulation.open_gripper()
            self.manipulation.clear_octomap()
            self.wait(2.0)
        return False

    def look_at_table(self) -> list:
        """Move to table_stare and return pickable detections, closest first."""
        if not self.go_to_table_stare():
            return []
        self.wait(self.settle_time)
        # The detector can lag a few seconds behind the camera after the arm moves
        for _ in range(6):
            status, detections = self.vision.detect_objects(timeout=5)
            if status == Status.EXECUTION_SUCCESS:
                break
            self.wait(2.0)
        else:
            return []
        names = [d.classname for d in detections]
        self.get_logger().info(f"Detections: {names}")
        pickable = [
            d
            for d in detections
            if d.classname in PICKABLE
            and self.attempts.get(d.classname, 0) < self.attempt_limit
            and self.on_dining_table(d)
        ]
        return sorted(pickable, key=lambda d: d.distance)

    def pick_and_place(self, name: str) -> str:
        """Pick one object from the dining table and place it on the side table."""
        self.attempts[name] = self.attempts.get(name, 0) + 1
        self.get_logger().info(f"Picking {name} (attempt {self.attempts[name]})")
        if self.manipulation.pick_object(name) != Status.EXECUTION_SUCCESS:
            self.manipulation.open_gripper()
            return "pick_failed"
        self.wait(1.0)
        if not self.holding:
            self.get_logger().warn(f"Gripper reports nothing held after picking {name}")
            self.manipulation.open_gripper()
            return "pick_empty"
        self.manipulation.move_joint_positions(
            joint_positions=SIDE_TABLE_STARE, velocity=0.5, degrees=True
        )
        self.wait(self.settle_time)
        if (
            self.manipulation.place(close_to=PLACE_ANCHOR, from_current=True)
            != Status.EXECUTION_SUCCESS
        ):
            self.get_logger().error(f"Place failed for {name}; opening the gripper")
            self.manipulation.open_gripper()
            return "place_failed"
        # Placed objects leave the table, so another object with the same label may still be picked
        self.attempts[name] = 0
        return "success"

    def run(self):
        self.get_logger().info("Sim pick and place started")
        self.manipulation.open_gripper()
        for _ in range(self.max_objects * self.attempt_limit):
            # Only finish once the table looks empty twice in a row
            targets = self.look_at_table() or self.look_at_table()
            if not targets:
                break
            name = targets[0].classname
            outcome = self.pick_and_place(name)
            self.results.append((name, outcome))
            self.get_logger().info(f"{name}: {outcome}")
            if sum(1 for _, o in self.results if o == "success") >= self.max_objects:
                break
        self.manipulation.move_to_position("table_stare")
        self.report()

    def report(self):
        successes = sum(1 for _, o in self.results if o == "success")
        self.get_logger().info("===== SIM PICK AND PLACE REPORT =====")
        for name, outcome in self.results:
            self.get_logger().info(f"  {name:12s} {outcome}")
        self.get_logger().info(
            f"  placed {successes} object(s) in {len(self.results)} attempt(s)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimPickAndPlaceTM()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
