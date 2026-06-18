#!/usr/bin/env python3

"""
End-to-end test: get washing-machine drum centroid via moondream, then move
the xArm to a pose at a fixed distance in front of the robot base, at the
height of the detected centroid. The robot does NOT try to reach the centroid
in 3D — it only matches its height.
"""

import sys

import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import PointStamped

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

FORWARD_DISTANCE = 0.50  # meters in front of base_link the EE is sent to
LATERAL = 0.0  # meters left/right of base_link
VELOCITY = 0.2
CAMERA_FLIP = False
SUBJECT = (
    "exact geometric center of the circular washing machine drum opening "
    "(the round hole in the front door where clothes go in); point at the "
    "middle of the circle, not the door, rim, glass, or     surrounding frame"
)

# Hardcoded "centroid" so we can exercise MoveToPose without ZED/moondream.
# move_to_height_in_front uses only point.z as the target height; the goal
# x/y come from FORWARD_DISTANCE/LATERAL.
HARDCODED_X = 3.0
HARDCODED_Y = 0.0
HARDCODED_Z = 1.00


class TestMoveToWashingMachine(Node):
    def __init__(self):
        super().__init__("test_move_to_washing_machine")
        self.vision = VisionTasks(self, task=Task.DEBUG)
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG)
        rclpy.spin_once(self, timeout_sec=1.0)
        self.vision.camera_upside_down(CAMERA_FLIP)

    def run(self) -> int:
        # Pre-pose only needed when the per-axis path constraint is enabled
        # (table_stare keeps the start state in roughly the constraint manifold).
        # With the scalar goal-only tolerance, leaving the arm in its current pose
        # gives the planner more flexibility — table_stare leaves the gripper
        # pointing 50° down, which forces a wrist flip to reach a horizontal goal.
        # Logger.info(self, "Pre-positioning to table_stare so gripper starts horizontal...")
        # pre_status = self.manipulation.move_joint_positions(
        #     named_position="table_stare", velocity=0.3
        # )
        # if pre_status != Status.EXECUTION_SUCCESS:
        #     Logger.error(self, "Pre-position to table_stare failed. Aborting.")
        #     return 3

        # --- HARDCODED fallback (kept commented for quick debugging without ZED) ---
        # Logger.info(self, "Using HARDCODED centroid (vision disabled)")
        # point = PointStamped()
        # point.header.frame_id = "base_link"
        # point.header.stamp = self.get_clock().now().to_msg()
        # point.point.x = HARDCODED_X
        # point.point.y = HARDCODED_Y
        # point.point.z = HARDCODED_Z
        # --- Vision path ---
        Logger.info(self, "Requesting washing machine hole centroid from moondream...")
        point = self.vision.get_moondream_point_3d(SUBJECT)
        if point is None:
            Logger.error(self, "No centroid returned from moondream. Aborting.")
            return 1

        p = point.point
        Logger.success(
            self,
            f"Centroid @ {point.header.frame_id}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})",
        )

        Logger.info(
            self,
            f"Commanding MoveToPose at (x={FORWARD_DISTANCE:.2f}, "
            f"y={LATERAL:.2f}, z=<centroid height>) in base_link",
        )
        status = self.manipulation.move_to_height_in_front(
            reference_point=point,
            forward_distance=FORWARD_DISTANCE,
            lateral=LATERAL,
            velocity=VELOCITY,
            tolerance_orientation=0.5,  # ~28° at goal — wide enough for IK sampler to hit
            # Per-axis path constraint in base_link: [rot_X, rot_Y, rot_Z].
            # X (wrist roll about approach) FREE.
            # Y (pitch up/down) wide (~57°) — table_stare starts with the gripper
            # pitched ~50° down. If Y is too tight, the START STATE itself violates
            # the constraint and the projection forces self-collisions.
            # Z (yaw) tight (~17°) — gripper must keep pointing forward, not sideways.
            # The scalar tolerance_orientation=0.2 above still pulls the GOAL to
            # horizontal regardless of Y slack.
            # per_axis_orientation_tolerance=(3.14159, 1.0, 0.3),
        )

        if status == Status.EXECUTION_SUCCESS:
            Logger.success(self, "Move to washing machine approach pose: SUCCESS")
            return 0
        Logger.error(self, f"Move failed with status={status}")
        return 2


def main(args=None):
    rclpy.init(args=args)
    node = TestMoveToWashingMachine()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        try:
            node.vision.camera_upside_down(False)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
