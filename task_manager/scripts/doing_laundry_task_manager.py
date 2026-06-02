#!/usr/bin/env python3
"""
Task Manager for Doing Laundry Task
"""

import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point  # noqa: F401
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from frida_constants.manipulation_constants import ZED_POINT_CLOUD_TOPIC
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.subtask_manager import SubtaskManager, Task

ATTEMPT_LIMIT = 3

# Height filter in link_base frame for basket rim.
# Robot mobile base ≈ 70cm; xArm sits on top → link_base ≈ 70cm above floor.
# Empirically the floor appears at z ≈ −0.53 in link_base (camera doesn't see all
# of the floor due to its tilt; full floor would be at −0.70).
# A floor-standing laundry basket has its rim ~25–35cm above floor → z ∈ [−0.45, −0.35].
# Allow a generous band that still excludes table-height furniture (z ≥ -0.10).
BASKET_HEIGHT_MIN = -0.60
BASKET_HEIGHT_MAX = -0.25

# Max XY distance from link_base origin to consider a point as basket
BASKET_MAX_DIST = 1.5

# xarm6 physical reach limit from link_base — xArm6 max is ~0.85m.
# Beyond this MoveIt cannot find an IK solution. Setting this higher just lets
# the script attempt a guaranteed-fail MoveIt call (does NOT extend the arm).
ARM_MAX_REACH = 0.85

# Subsample step when scanning the point cloud (higher = faster, less precise)
CLOUD_SAMPLE_STEP = 4

# Density clustering for basket isolation (DBSCAN-lite, region growing)
# The basket rim is a dense elevated contour; floor noise is sparse.
CLUSTER_RADIUS = 0.08  # neighbor radius per growth step (m)
CLUSTER_EXPANSIONS = 25  # iterations of growth — basket perimeter / radius
MIN_CLUSTER_SIZE = 80  # basket rim contour easily has 100+ pts when fully grown
MIN_CLUSTER_EXTENT = 0.18  # basket spans ≥ ~20cm; allow some tolerance
MAX_CLUSTER_EXTENT = 0.80  # > this is wall/floor, not a basket

# Top-down grasp parameters
PRE_GRASP_HEIGHT_ABOVE_RIM = 0.15  # pre-grasp hovers this high (m) above rim
GRASP_DEPTH_BELOW_RIM = 0.03  # grasp_frame descends this much below rim (m) to wrap it
PRE_GRASP_VELOCITY = 0.30
DESCENT_VELOCITY = 0.12
# Approach tilt: 0° = pure top-down (gripper Z straight down). 90° = horizontal
# approach (gripper Z forward toward basket). Higher tilt = easier IK at long
# reach because arm doesn't need to fold over the target. Tune in [30, 70].
APPROACH_PITCH_DEG = 60.0
LIFT_VELOCITY = 0.20

# Base approach parameters: if basket is farther than PICKABLE_RANGE, the base
# rolls (open-loop forward or backward depending on basket side) until basket
# is within range. Uses closed-loop TF feedback from odom for accurate motion.
PICKABLE_RANGE = 0.55  # m — target horizontal dist (XY) from link_base to basket
APPROACH_LINEAR_SPEED = 0.18  # m/s — base velocity command (real may be lower if filtered)
APPROACH_TIMEOUT_FACTOR = 6.0  # max time = distance/speed * this factor (safety vs filtering)
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
CMD_VEL_TOPIC = "/cmd_vel"


class DoingLaundryTM(Node):
    """Task Manager for Doing Laundry"""

    class TaskStates:
        WAIT_FOR_BUTTON = "WAIT_FOR_BUTTON"
        START = "START"
        NAVIGATE_TO_BASKET = "NAVIGATE_TO_BASKET"
        ROTATE_BEHIND_BASKET = "ROTATE_BEHIND_BASKET"
        LOOK_BACK_AND_SCAN = "LOOK_BACK_AND_SCAN"
        APPROACH_BASKET = "APPROACH_BASKET"
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
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self.create_publisher(Marker, "/laundry/basket_grasp_marker", marker_qos)
        self._cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.approach_attempts = 0

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

    def _approach_base_backward(self, distance: float) -> bool:
        """Drive the base backward by `distance` meters using direct /cmd_vel.

        Robot's back faces the basket (after the 180° rotation), so backward motion
        closes the gap. Uses timed open-loop control — no odometry feedback, but
        accurate enough for small approaches (<0.8m) at low speed.

        Returns True if the motion was commanded, False if distance was invalid.
        """
        if distance <= 0.02:
            Logger.info(self, "Approach distance too small, skipping base motion.")
            return True

        # Cap to a safe maximum so a bad estimate never sends the robot flying
        distance = min(distance, 1.0)
        duration = distance / APPROACH_LINEAR_SPEED

        Logger.info(
            self,
            f"Base approach: moving backward {distance:.2f}m at {APPROACH_LINEAR_SPEED:.2f}m/s "
            f"(duration ~{duration:.1f}s)",
        )

        twist = Twist()
        twist.linear.x = -APPROACH_LINEAR_SPEED  # backward in base_link

        # Publish at 20 Hz for the duration, then send a zero-stop
        start = self.get_clock().now()
        end = start + Duration(seconds=duration)
        rate_sec = 0.05
        while self.get_clock().now() < end and rclpy.ok():
            self._cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=rate_sec)

        # Hard stop
        for _ in range(5):
            self._cmd_vel_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

        Logger.success(self, "Base approach complete.")
        return True

    # ------------------------------------------------------------------ point cloud

    def _cloud_cb(self, msg: PointCloud2):
        self._latest_cloud = msg

    def _extract_basket_center_from_cloud(self) -> PointStamped:
        """Isolate the basket as the densest elevated cluster and return its XY centroid + rim Z.

        Pipeline:
          1. Parse + subsample ZED point cloud.
          2. Transform all points to link_base (xarm6 base frame).
          3. Filter by height (basket-rim band) and max distance from arm base.
          4. Region-grow clustering from nearest seed: any cluster < MIN_CLUSTER_SIZE
             is discarded as sparse noise (floor specks, wall edges).
          5. From the largest dense cluster (the basket rim contour), compute:
                XY centroid → basket center in plan view
                Z = max(cluster.z) → top of rim, used as grasp target

        Returns PointStamped in link_base, or None if no valid basket cluster found.
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

        # Get transform from camera frame to link_base (xarm6 base)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                "link_base",
                cloud.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as e:
            Logger.error(self, f"TF cloud→link_base failed: {e}")
            return None

        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        Logger.info(
            self,
            f"[DBG] TF {cloud.header.frame_id}→link_base: "
            f"t=({t.x:.3f},{t.y:.3f},{t.z:.3f}) "
            f"q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})",
        )

        # Build rotation matrix from quaternion
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

        Logger.info(
            self,
            f"[DBG] Cloud in link_base — {len(xyz_bl)} pts — "
            f"x:[{xyz_bl[:,0].min():.2f},{xyz_bl[:,0].max():.2f}] "
            f"y:[{xyz_bl[:,1].min():.2f},{xyz_bl[:,1].max():.2f}] "
            f"z:[{xyz_bl[:,2].min():.2f},{xyz_bl[:,2].max():.2f}]",
        )

        # Filter by height: basket rim range in link_base frame
        height_mask = (xyz_bl[:, 2] >= BASKET_HEIGHT_MIN) & (xyz_bl[:, 2] <= BASKET_HEIGHT_MAX)
        xyz_bl = xyz_bl[height_mask]

        if len(xyz_bl) == 0:
            Logger.warn(
                self,
                f"No points in basket height range [{BASKET_HEIGHT_MIN}, {BASKET_HEIGHT_MAX}]m",
            )
            return None

        Logger.info(
            self,
            f"[DBG] After height filter: {len(xyz_bl)} pts — "
            f"x:[{xyz_bl[:,0].min():.2f},{xyz_bl[:,0].max():.2f}] "
            f"y:[{xyz_bl[:,1].min():.2f},{xyz_bl[:,1].max():.2f}] "
            f"z:[{xyz_bl[:,2].min():.2f},{xyz_bl[:,2].max():.2f}]",
        )

        # Filter by max distance from robot
        dists = np.linalg.norm(xyz_bl, axis=1)
        dist_mask = dists <= BASKET_MAX_DIST
        xyz_bl = xyz_bl[dist_mask]
        dists = dists[dist_mask]

        if len(xyz_bl) == 0:
            Logger.warn(self, f"No points within {BASKET_MAX_DIST}m after height filter")
            return None

        # Region-grow clustering: from each unvisited seed (closest first), expand a
        # cluster by repeatedly adding neighbors within CLUSTER_RADIUS. Keep the
        # largest cluster across all seeds. Sparse seeds (clusters smaller than
        # MIN_CLUSTER_SIZE) are skipped — they're noise, not the basket.
        # Iterate seeds from CLOSEST to farthest. As soon as we find a cluster
        # with >= MIN_CLUSTER_SIZE points AND a bounded XY extent (basket is compact,
        # walls/floors are huge), accept it. This prefers the basket sitting close
        # to the robot over distant furniture even if the furniture cluster is larger.
        sorted_idxs = np.argsort(dists)
        visited = np.zeros(len(xyz_bl), dtype=bool)
        best_cluster_pts = None
        best_dist = float("inf")

        for seed in sorted_idxs:
            if visited[seed]:
                continue
            cluster_mask = np.zeros(len(xyz_bl), dtype=bool)
            cluster_mask[seed] = True
            # True DBSCAN-style growth: a point joins if its distance to ANY
            # cluster point is ≤ CLUSTER_RADIUS. This captures the full basket
            # rim contour even though the interior is hollow (a centroid-based
            # growth gets stuck once the centroid lands inside the hollow).
            for _ in range(CLUSTER_EXPANSIONS):
                non_cluster = np.where(~cluster_mask)[0]
                if len(non_cluster) == 0:
                    break
                # Safety: if cluster already exceeds basket plausibility, stop early
                if int(cluster_mask.sum()) > 1500:
                    break
                nc_pts = xyz_bl[non_cluster]
                cluster_pts = xyz_bl[cluster_mask]
                # Pairwise squared distance: (N,1,3)-(1,M,3) → (N,M); compare squared
                diffs = nc_pts[:, None, :] - cluster_pts[None, :, :]
                min_sq = (diffs * diffs).sum(axis=2).min(axis=1)
                new_idxs = non_cluster[min_sq <= CLUSTER_RADIUS * CLUSTER_RADIUS]
                if len(new_idxs) == 0:
                    break
                cluster_mask[new_idxs] = True

            visited |= cluster_mask
            size = int(cluster_mask.sum())
            if size < MIN_CLUSTER_SIZE:
                continue

            cluster_pts = xyz_bl[cluster_mask]
            x_range = float(cluster_pts[:, 0].ptp())
            y_range = float(cluster_pts[:, 1].ptp())
            # Reject too-wide clusters (walls/floor) or too-small clusters (random boxes/objects).
            # A real laundry basket spans ~30–45cm on its widest side.
            if x_range > MAX_CLUSTER_EXTENT or y_range > MAX_CLUSTER_EXTENT:
                Logger.info(
                    self,
                    f"[DBG] Skipping wide cluster ({size} pts, "
                    f"x_range={x_range:.2f} y_range={y_range:.2f}) — looks like wall/floor",
                )
                continue
            if max(x_range, y_range) < MIN_CLUSTER_EXTENT:
                Logger.info(
                    self,
                    f"[DBG] Skipping small cluster ({size} pts, "
                    f"x_range={x_range:.2f} y_range={y_range:.2f}) — too small to be basket",
                )
                continue

            seed_dist = float(dists[seed])
            Logger.info(
                self,
                f"[DBG] Candidate cluster: {size} pts at dist={seed_dist:.2f}m, "
                f"extents=({x_range:.2f},{y_range:.2f})",
            )

            if seed_dist < best_dist:
                best_dist = seed_dist
                best_cluster_pts = cluster_pts
                # First good candidate from the closest seed is normally enough.
                # Stop once we have ANY valid basket-like cluster.
                break

        if best_cluster_pts is None:
            Logger.warn(
                self,
                f"No basket-like cluster found (min size={MIN_CLUSTER_SIZE}, "
                "max extent=0.80m). Check BASKET_HEIGHT_MIN/MAX and lighting.",
            )
            return None

        best_size = len(best_cluster_pts)
        # XY centroid = basket center in plan view
        cx = float(np.mean(best_cluster_pts[:, 0]))
        cy = float(np.mean(best_cluster_pts[:, 1]))
        # Rim height: 90th-percentile Z (robust to outliers above the rim)
        rim_z = float(np.percentile(best_cluster_pts[:, 2], 90))

        Logger.success(
            self,
            f"Basket isolated: {best_size} pts, center=({cx:.3f},{cy:.3f}) "
            f"rim_z={rim_z:.3f} (x_range={best_cluster_pts[:,0].ptp():.2f}, "
            f"y_range={best_cluster_pts[:,1].ptp():.2f})",
        )

        # DEBUG: compare camera-frame distance vs link_base-frame distance
        centroid_bl = np.array([cx, cy, float(np.mean(best_cluster_pts[:, 2]))])
        cam_dist = float(np.linalg.norm(centroid_bl - translation))
        cam_horiz = float(np.linalg.norm((centroid_bl - translation)[:2]))
        bl_dist = float(np.linalg.norm(centroid_bl))
        bl_horiz = float(np.linalg.norm(centroid_bl[:2]))
        Logger.info(
            self,
            f"[DBG] Camera origin in link_base: ({translation[0]:.2f},{translation[1]:.2f},{translation[2]:.2f}) | "
            f"Dist from CAMERA: {cam_dist:.2f}m (horiz {cam_horiz:.2f}m) | "
            f"Dist from LINK_BASE: {bl_dist:.2f}m (horiz {bl_horiz:.2f}m)",
        )

        pt = PointStamped()
        pt.header.frame_id = "link_base"
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = cx
        pt.point.y = cy
        pt.point.z = rim_z
        self._publish_marker(pt, color=(1.0, 0.5, 0.0), marker_id=0)
        return pt

    def _publish_marker(self, point: PointStamped, color=(1.0, 0.0, 0.0), marker_id: int = 0):
        m = Marker()
        m.header = point.header
        m.ns = "basket"
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = point.point.x
        m.pose.position.y = point.point.y
        m.pose.position.z = point.point.z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.08
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        m.lifetime.sec = 10
        self._marker_pub.publish(m)

    def _publish_pose_marker(self, pose: PoseStamped, color=(0.0, 1.0, 1.0), marker_id: int = 1):
        m = Marker()
        m.header = pose.header
        m.ns = "basket"
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = 0.15
        m.scale.y = 0.02
        m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        m.lifetime.sec = 10
        self._marker_pub.publish(m)

    # ------------------------------------------------------------------ helpers

    def _topdown_grasp_pose(self, center: PointStamped, z_offset: float) -> PoseStamped:
        """Build a top-down grasp PoseStamped above (or at) the basket rim.

        The gripper_grasp_frame Z-axis points straight DOWN in link_base (i.e., -Z world),
        so a positive z_offset above rim is the pre-grasp, and a negative z_offset
        (just below rim) is the grasp pose that wraps the rim.

        Yaw is set so the gripper fingers open across the line from the arm to the rim
        center (perpendicular to that line = the "natural" approach direction).

        Args:
            center: PointStamped (link_base) with XY = basket center, Z = rim height.
            z_offset: meters above rim (positive = pre-grasp, negative = inside basket).

        Returns:
            PoseStamped in link_base, or None if target is outside ARM_MAX_REACH.
        """
        bx = center.point.x
        by = center.point.y
        rim_z = center.point.z
        target_z = rim_z + z_offset

        reach = math.sqrt(bx * bx + by * by + target_z * target_z)
        Logger.info(
            self,
            f"Top-down target ({bx:.2f},{by:.2f},{target_z:.2f}) "
            f"reach={reach:.2f}m — letting MoveIt try IK",
        )

        # Slanted approach orientation: quaternion_from_euler(roll, 0, yaw)
        # where roll = -(pi/2 - tilt). tilt=0 → pure top-down (Z down).
        # tilt=pi/2 → horizontal (Z forward). Larger tilt makes IK easier at long
        # reach because the arm doesn't need to fold the wrist 90° downward.
        yaw = math.atan2(by, bx)
        tilt = math.radians(APPROACH_PITCH_DEG)
        roll = -(math.pi / 2 - tilt)
        half_r = roll / 2.0
        half_y = yaw / 2.0
        sr = math.sin(half_r)
        cr = math.cos(half_r)
        sy = math.sin(half_y)
        cy = math.cos(half_y)
        # ZYX intrinsic (roll, 0, yaw) with pitch=0:
        qx = sr * cy
        qy = -sr * sy
        qz = cr * sy
        qw = cr * cy

        pose = PoseStamped()
        pose.header.frame_id = "link_base"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = bx
        pose.pose.position.y = by
        pose.pose.position.z = target_z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        Logger.info(
            self,
            f"Slanted pose: pos=({bx:.3f},{by:.3f},{target_z:.3f}) "
            f"yaw={math.degrees(yaw):.1f}° tilt={APPROACH_PITCH_DEG:.0f}° "
            f"z_offset={z_offset:+.3f}m reach={reach:.3f}m",
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

            Logger.info(self, "Extracting basket centroid + rim via density clustering.")
            grasp_point = self._extract_basket_center_from_cloud()

            if grasp_point is not None:
                horiz_dist = math.sqrt(grasp_point.point.x**2 + grasp_point.point.y**2)
                Logger.success(
                    self,
                    f"Basket grasp point found: ({grasp_point.point.x:.3f}, "
                    f"{grasp_point.point.y:.3f}, {grasp_point.point.z:.3f}) "
                    f"horiz_dist={horiz_dist:.3f}m",
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

            def _rescan(reason: str):
                Logger.error(self, f"{reason} Re-scanning.")
                self.basket_grasp_point = None
                self.scan_attempts = 0
                self.current_state = DoingLaundryTM.TaskStates.LOOK_BACK_AND_SCAN

            Logger.info(self, "Opening gripper before top-down approach.")
            self.subtask_manager.manipulation.open_gripper()

            # === Stage 1: pre-grasp (hover above the rim) ============================
            pre_grasp = self._topdown_grasp_pose(
                self.basket_grasp_point, z_offset=PRE_GRASP_HEIGHT_ABOVE_RIM
            )
            if pre_grasp is None:
                Logger.error(
                    self,
                    "Pre-grasp pose unreachable — basket is physically out of arm "
                    "reach. Move the basket closer and restart the task.",
                )
                self.current_state = DoingLaundryTM.TaskStates.END
                return
            self._publish_pose_marker(pre_grasp, color=(0.0, 1.0, 1.0), marker_id=1)

            Logger.info(
                self,
                f"[1/3] PRE-GRASP: hovering at z={pre_grasp.pose.position.z:.3f} "
                f"(rim_z + {PRE_GRASP_HEIGHT_ABOVE_RIM:.2f}m)",
            )
            result = self.subtask_manager.manipulation.move_arm_to_pose(
                pre_grasp, velocity=PRE_GRASP_VELOCITY
            )
            if result != Status.EXECUTION_SUCCESS:
                Logger.error(
                    self,
                    "Pre-grasp MoveIt failed — the basket is beyond the xArm6's "
                    "physical reach (~0.85m). Move the basket closer.",
                )
                self.current_state = DoingLaundryTM.TaskStates.END
                return

            # === Stage 2: descend to rim (grasp pose) ================================
            grasp = self._topdown_grasp_pose(
                self.basket_grasp_point, z_offset=-GRASP_DEPTH_BELOW_RIM
            )
            if grasp is None:
                Logger.error(
                    self,
                    "Grasp pose unreachable — basket out of arm reach. "
                    "Move the basket closer and restart.",
                )
                self.current_state = DoingLaundryTM.TaskStates.END
                return
            self._publish_pose_marker(grasp, color=(1.0, 0.0, 1.0), marker_id=2)

            Logger.info(
                self,
                f"[2/3] DESCEND: lowering to z={grasp.pose.position.z:.3f} "
                f"(rim_z − {GRASP_DEPTH_BELOW_RIM:.2f}m) to wrap rim",
            )
            result = self.subtask_manager.manipulation.move_arm_to_pose(
                grasp, velocity=DESCENT_VELOCITY
            )
            if result != Status.EXECUTION_SUCCESS:
                _rescan("Descent failed.")
                return

            # === Stage 3: close gripper around rim ===================================
            Logger.info(self, "[3/3] CLOSE gripper around basket rim.")
            self.subtask_manager.manipulation.close_gripper()

            # Lift back to pre-grasp height so we don't drag the basket on the floor
            Logger.info(self, "Lifting basket to pre-grasp height.")
            self.subtask_manager.manipulation.move_arm_to_pose(pre_grasp, velocity=LIFT_VELOCITY)

            # Compact carry pose for navigation
            Logger.info(self, "Moving to basket_hold_back_pose for transport.")
            self.subtask_manager.manipulation.move_to_position("basket_hold_back_pose")

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
