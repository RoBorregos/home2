#!/usr/bin/env python3
"""
Task Manager for Doing Laundry Task
"""

import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point  # noqa: F401
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import PointCloud2
from frida_constants.manipulation_constants import ZED_POINT_CLOUD_TOPIC
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3

# Height filter in base_link frame for basket rim (meters above floor)
BASKET_HEIGHT_MIN = 0.05
BASKET_HEIGHT_MAX = 0.50

# Max distance from base_link origin to consider a point as basket
BASKET_MAX_DIST = 1.5

# Subsample step when scanning the point cloud (higher = faster, less precise)
CLOUD_SAMPLE_STEP = 6


class DoingLaundryTM(Node):
    """Task Manager for Doing Laundry"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        NAVIGATE_TO_BASKET = "NAVIGATE_TO_BASKET"
        ROTATE_BEHIND_BASKET = "ROTATE_BEHIND_BASKET"
        LOOK_BACK_AND_SCAN = "LOOK_BACK_AND_SCAN"
        PICK_LAUNDRY = "PICK_LAUNDRY"
        NAVIGATE_TO_LAUNDRY_TABLE = "NAVIGATE_TO_LAUNDRY_TABLE"
        UNLOAD_LAUNDRY = "UNLOAD_LAUNDRY"
        END = "END"

    def __init__(self):
        super().__init__("doing_laundry_task_manager")
        self.subtask_manager = SubtaskManager(
            self, task=Task.DOING_LAUNDRY, mock_areas=["navigation", "vision", "hri"]
        )
        self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN
        self.running_task = True
        self.basket_grasp_point: PointStamped = None
        self.scan_attempts = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._latest_cloud: PointCloud2 = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, ZED_POINT_CLOUD_TOPIC, self._cloud_cb, qos)

        self.subtask_manager.manipulation.move_to_position("nav_pose")
        Logger.info(self, "DoingLaundryTM has started.")

    # ------------------------------------------------------------------ navigation

    def navigate_to(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate to location, resetting arm to nav_pose first."""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        self.subtask_manager.manipulation.move_to_position("nav_pose")
        if say:
            Logger.info(self, f"Moving to {location}")
            self.subtask_manager.hri.say(f"Navigating to {location}.", wait=False)
        return self.subtask_manager.nav.move_to_location(location, sublocation)

    def navigate_holding(self, location: str, sublocation: str = "", say: bool = True):
        """Navigate while holding basket — does NOT reset arm to nav_pose."""
        self.subtask_manager.vision.deactivate_face_recognition()
        self.subtask_manager.manipulation.follow_face(False)
        if say:
            Logger.info(self, f"Carrying basket to {location}")
            self.subtask_manager.hri.say(f"Carrying basket to {location}.", wait=False)
        return self.subtask_manager.nav.move_to_location(location, sublocation)

    def _rotate_180(self):
        """Rotate 180° in place so the robot's back faces the basket."""
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
        except TransformException as e:
            Logger.error(self, f"TF base_link→map failed for rotation: {e}")
            return Status.EXECUTION_ERROR

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y
        q = robot_tf.transform.rotation
        current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2),
        )
        new_yaw = current_yaw + math.pi

        Logger.info(
            self,
            f"Rotating 180°: {math.degrees(current_yaw):.1f}° → {math.degrees(new_yaw):.1f}°",
        )
        self.subtask_manager.hri.say("Turning to face basket.", wait=False)
        status, error = self.subtask_manager.nav.navigate_to_pose(rx, ry, new_yaw)
        if status != Status.EXECUTION_SUCCESS:
            Logger.error(self, f"Rotation failed: {error}")
        return status

    # ------------------------------------------------------------------ point cloud

    def _cloud_cb(self, msg: PointCloud2):
        self._latest_cloud = msg

    def _nearest_basket_point_from_cloud(self) -> PointStamped:
        """Find the nearest point cloud point in base_link that is at basket height.

        Transforms the full cloud to base_link, filters by height and distance,
        and returns the closest point to the robot. No vision required.

        Returns PointStamped in base_link, or None if no valid points found.
        """
        cloud = self._latest_cloud
        if cloud is None or cloud.height == 0 or cloud.width == 0:
            Logger.warn(self, "No point cloud received yet")
            return None

        h, w = cloud.height, cloud.width
        floats_per_point = cloud.point_step // 4

        # Parse cloud data and subsample
        data = np.frombuffer(cloud.data, dtype=np.float32).reshape(h * w, floats_per_point)
        data = data[::CLOUD_SAMPLE_STEP]
        xyz_cam = data[:, :3]

        # Filter out NaN/inf and points with no valid depth
        valid = np.all(np.isfinite(xyz_cam), axis=1) & (xyz_cam[:, 2] > 0.05)
        xyz_cam = xyz_cam[valid]

        if len(xyz_cam) == 0:
            Logger.warn(self, "Point cloud has no valid points after filtering")
            return None

        # Get transform from camera frame to base_link
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                "base_link",
                cloud.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as e:
            Logger.error(self, f"TF cloud→base_link failed: {e}")
            return None

        # Build rotation matrix from quaternion
        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array(
            [
                [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
                [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
            ]
        )
        translation = np.array([t.x, t.y, t.z])
        xyz_bl = (R @ xyz_cam.T).T + translation

        # Filter by height: basket rim range above floor in base_link
        height_mask = (xyz_bl[:, 2] >= BASKET_HEIGHT_MIN) & (xyz_bl[:, 2] <= BASKET_HEIGHT_MAX)
        xyz_bl = xyz_bl[height_mask]

        if len(xyz_bl) == 0:
            Logger.warn(
                self,
                f"No points in basket height range [{BASKET_HEIGHT_MIN}, {BASKET_HEIGHT_MAX}]m",
            )
            return None

        # Filter by max distance from robot
        dists = np.linalg.norm(xyz_bl, axis=1)
        dist_mask = dists <= BASKET_MAX_DIST
        xyz_bl = xyz_bl[dist_mask]
        dists = dists[dist_mask]

        if len(xyz_bl) == 0:
            Logger.warn(self, f"No points within {BASKET_MAX_DIST}m after height filter")
            return None

        idx = int(np.argmin(dists))
        nearest = xyz_bl[idx]

        pt = PointStamped()
        pt.header.frame_id = "base_link"
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = float(nearest[0])
        pt.point.y = float(nearest[1])
        pt.point.z = float(nearest[2])

        Logger.info(
            self,
            f"Nearest basket point in base_link: ({nearest[0]:.3f}, {nearest[1]:.3f}, "
            f"{nearest[2]:.3f}) dist={dists[idx]:.3f}m",
        )
        return pt

    # ------------------------------------------------------------------ helpers

    def _basket_approach_pose(self, point_stamped: PointStamped) -> PoseStamped:
        """Compute a PoseStamped for arm approach to basket.

        Transforms the given PointStamped to base_link and builds a
        horizontal-approach orientation (gripper pointing toward basket).

        Returns PoseStamped in base_link, or None on TF failure.
        """
        try:
            basket_bl = self.tf_buffer.transform(
                point_stamped, "base_link", timeout=Duration(seconds=1.0)
            )
        except TransformException as e:
            Logger.error(self, f"TF basket→base_link failed: {e}")
            return None

        bx = basket_bl.point.x
        by = basket_bl.point.y
        bz = basket_bl.point.z

        # quaternion_from_euler(roll=0, pitch=pi/2, yaw=yaw) — computed inline
        yaw = math.atan2(by, bx)
        cp = math.cos(math.pi / 4)
        sp = math.sin(math.pi / 4)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)
        qw = cp * cy
        qx = -sp * sy
        qy = sp * cy
        qz = cp * sy

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = bx
        pose.pose.position.y = by
        pose.pose.position.z = bz
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        Logger.info(
            self,
            f"Basket approach pose: ({bx:.3f}, {by:.3f}, {bz:.3f}) yaw={math.degrees(yaw):.1f}°",
        )
        return pose

    # ------------------------------------------------------------------ FSM

    def run(self):
        if self.current_state == DoingLaundryTM.TaskStates.WAIT_FOR_BUTTON:
            Logger.state(self, "Waiting for start button...")
            self.subtask_manager.hri.say("Waiting for start button to be pressed.", wait=False)

            while not self.subtask_manager.hri.start_button_clicked:
                rclpy.spin_once(self, timeout_sec=0.1)
            Logger.success(self, "Start button pressed, Doing Laundry task will begin now")
            self.current_state = DoingLaundryTM.TaskStates.START

        elif self.current_state == DoingLaundryTM.TaskStates.START:
            Logger.state(self, "Starting Doing Laundry Task")
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_BASKET:
            Logger.info(self, "Navigating to basket area")
            status, error = self.navigate_to("laundry", "basket_view")

            if status == Status.EXECUTION_SUCCESS:
                self.current_state = DoingLaundryTM.TaskStates.ROTATE_BEHIND_BASKET
            else:
                Logger.error(self, f"Navigation failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.ROTATE_BEHIND_BASKET:
            Logger.info(self, "Rotating 180° so back faces basket.")
            status = self._rotate_180()

            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Rotation complete. Back now faces basket.")
                self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN
            else:
                Logger.error(self, "Rotation failed. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN:
            Logger.info(self, "Moving to look_back_stare to scan basket from behind.")
            result = self.subtask_manager.manipulation.move_to_position("look_back_stare")

            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)

            Logger.info(self, "Scanning for nearest basket point via point cloud.")
            grasp_point = self._nearest_basket_point_from_cloud()

            if grasp_point is not None:
                Logger.success(
                    self,
                    f"Basket grasp point found: ({grasp_point.point.x:.3f}, "
                    f"{grasp_point.point.y:.3f}, {grasp_point.point.z:.3f})",
                )
                self.basket_grasp_point = grasp_point
                self.current_state = DoingLaundryTM.TaskStates.PICK_LAUNDRY
            else:
                self.scan_attempts += 1
                if self.scan_attempts >= ATTEMPT_LIMIT:
                    Logger.error(
                        self, "Could not find basket in point cloud after max attempts. Ending."
                    )
                    self.current_state = DoingLaundryTM.TaskStates.END
                else:
                    Logger.warn(
                        self,
                        f"No basket point found (attempt {self.scan_attempts}), retrying...",
                    )

        elif self.current_state == DoingLaundryTM.TaskStates.PICK_LAUNDRY:
            if self.basket_grasp_point is None:
                Logger.error(self, "No basket grasp point available. Re-scanning.")
                self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN
                return

            Logger.info(self, "Opening gripper before approach.")
            self.subtask_manager.manipulation.open_gripper()

            Logger.info(self, "Computing basket approach pose.")
            target_pose = self._basket_approach_pose(self.basket_grasp_point)
            if target_pose is None:
                Logger.error(self, "Could not compute basket approach pose. Re-scanning.")
                self.basket_grasp_point = None
                self.scan_attempts = 0
                self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN
                return

            Logger.info(self, "Moving arm to basket via move_arm_to_pose.")
            result = self.subtask_manager.manipulation.move_arm_to_pose(target_pose, velocity=0.3)

            if result != Status.EXECUTION_SUCCESS:
                Logger.error(self, "move_arm_to_pose failed. Re-scanning.")
                self.basket_grasp_point = None
                self.scan_attempts = 0
                self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN
                return

            Logger.info(self, "Closing gripper to grab basket.")
            self.subtask_manager.manipulation.close_gripper()

            Logger.success(self, "Basket grabbed. Heading to laundry table.")
            self.current_state = DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE

        elif self.current_state == DoingLaundryTM.TaskStates.NAVIGATE_TO_LAUNDRY_TABLE:
            Logger.info(self, "Navigating to laundry table while holding basket.")
            status, error = self.navigate_holding("laundry", "laundry_table")

            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Reached laundry table.")
                self.current_state = DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY
            else:
                Logger.error(self, f"Navigation to table failed: {error}. Retrying...")

        elif self.current_state == DoingLaundryTM.TaskStates.UNLOAD_LAUNDRY:
            Logger.info(self, "Opening gripper to release basket at table.")
            self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say("Basket delivered to the table.", wait=False)
            self.current_state = DoingLaundryTM.TaskStates.END

        elif self.current_state == DoingLaundryTM.TaskStates.END:
            Logger.state(self, "Ending task")
            self.subtask_manager.hri.say("Laundry task finished. I will rest now.")
            self.running_task = False


def main(args=None):
    rclpy.init(args=args)
    node = DoingLaundryTM()
    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
