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
from geometry_msgs.msg import PointStamped

from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks

# TODO: re-enable vision when ZED is back
# from task_manager.subtask_managers.vision_tasks import VisionTasks
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
        # TODO: re-enable vision when ZED is back
        # self.vision = VisionTasks(self, task=Task.DEBUG)
        self.manipulation = ManipulationTasks(self, task=Task.DEBUG)
        rclpy.spin_once(self, timeout_sec=1.0)
        # self.vision.camera_upside_down(CAMERA_FLIP)

    def run(self) -> int:
        # Pre-pose: move the arm to a horizontal-forward stare BEFORE the path
        # constraint is enforced. The per-axis orientation constraint is applied
        # to every state in the trajectory; if the start state already satisfies
        # it, the planner doesn't have to fight contortions/self-collisions.
        Logger.info(self, "Pre-positioning to table_stare so gripper starts horizontal...")
        pre_status = self.manipulation.move_joint_positions(
            named_position="table_stare", velocity=0.3
        )
        if pre_status != Status.EXECUTION_SUCCESS:
            Logger.error(self, "Pre-position to table_stare failed. Aborting.")
            return 3

        Logger.info(self, "Using HARDCODED centroid (vision disabled)")
        point = PointStamped()
        point.header.frame_id = "base_link"
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = HARDCODED_X
        point.point.y = HARDCODED_Y
        point.point.z = HARDCODED_Z
        # TODO: re-enable vision when ZED is back
        # point = self.vision.get_moondream_point_3d(SUBJECT)
        # if point is None:
        #     Logger.error(self, "No centroid returned from moondream. Aborting.")
        #     return 1

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
            tolerance_orientation=0.2,
            # Per-axis path constraint in base_link: [rot_X, rot_Y, rot_Z].
            # X (wrist roll about approach) FREE; Y (pitch up/down) and Z (yaw)
            # at ~11° — gripper stays ~horizontal forward but planner has slack
            # so the goal sampler can find valid IK and the path can avoid
            # self-collisions. Tighten back to 0.05 only if the result looks tilted.
            # per_axis_orientation_tolerance=(3.14159, 0.2, 0.2),
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
        # TODO: re-enable vision when ZED is back
        # try:
        #     node.vision.camera_upside_down(False)
        # except Exception:
        #     pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
