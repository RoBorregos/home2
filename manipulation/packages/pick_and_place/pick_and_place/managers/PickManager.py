from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import PickPerceptionService, DetectionHandler
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import SetBool
from pick_and_place.utils.grasp_utils import get_grasps
from pick_and_place.utils.perception_utils import (
    get_object_cluster,
    get_object_detection,
    point_in_range,
)
from pick_and_place.utils.pointcloud_completion import complete_cluster
from pick_and_place.utils.dense_cloud import make_dense_cluster
from pick_and_place.utils.grasp_database import GraspDatabase
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from frida_interfaces.action import PickMotion
from frida_interfaces.msg import PickResult
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)
from frida_constants.manipulation_constants import (
    PICK_MAX_DISTANCE,
    CUTLERY_NAMES,
)
from frida_constants.vision_constants import (
    DEPTH_IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
)
from typing import Tuple
import time
import copy
from scipy.spatial.transform import Rotation as R
from sensor_msgs_py import point_cloud2
import numpy as np
from pick_and_place.utils.perception_utils import get_object_point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


CFG_PATHS = [
    [
        "/workspace/src/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper_testing.cfg",
        True,
    ],
    [
        "/workspace/src/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper.cfg",
        False,
    ],
]


# FINE HEIGHT ADJUSTMENT FOR CUTLERY
# If it's floating, use negative values (e.g. -0.02)
# If it hits too hard, use positive values (e.g. 0.02)
FLAT_GRASP_Z_TWEAK = 0.076

# Timeout waiting for the flat_grasp_estimator to publish a pose (seconds)
FLAT_GRASP_TIMEOUT = 5.0

# Number of poses to collect and average for stable Z
FLAT_GRASP_SAMPLES = 10

# Time to collect samples (seconds)
FLAT_GRASP_SAMPLE_WINDOW = 3.0


def is_cutlery(object_name: str) -> bool:
    if object_name is None:
        return False
    return object_name.lower() in CUTLERY_NAMES


class PickManager:
    node = None

    def __init__(self, node):
        self.node = node

        self.latest_flat_grasp = None
        self._collecting_samples = False
        self._grasp_samples = []

        # Grasp database for pre-computed grasps
        import os
        db_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "grasp_database",
        )
        self.grasp_db = GraspDatabase(db_path, logger=self.node.get_logger())
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Dense cloud: depth image + camera info subscribers
        self._latest_depth = None
        self._camera_info = None

        depth_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.node.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self._depth_cb, depth_qos
        )
        self.node.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._cam_info_cb, depth_qos
        )

        self._debug_cluster_pub = self.node.create_publisher(
            PointCloud2, "/manipulation/debug/cluster_raw", 1
        )
        self._debug_completed_pub = self.node.create_publisher(
            PointCloud2, "/manipulation/debug/cluster_completed", 1
        )

        self.node.create_subscription(
            PoseStamped, "/manipulation/flat_grasp_pose", self.flat_grasp_callback, 10
        )

    def _depth_cb(self, msg):
        self._latest_depth = msg

    def _cam_info_cb(self, msg):
        self._camera_info = msg

    def _get_tf_matrix(self, source_frame: str, target_frame: str):
        """Get 4x4 homogeneous transform from source to target frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            mat = np.eye(4)
            mat[:3, :3] = rot
            mat[:3, 3] = [t.x, t.y, t.z]
            return mat
        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def _try_dense_cloud(self, object_name: str, table_z: float | None = None):
        """Try to build a dense cloud from depth image + detection bbox.

        Returns (PointCloud2, min_z, height) or (None, 0, 0) on failure.
        """
        if self._latest_depth is None or self._camera_info is None:
            return None, 0.0, 0.0

        detection = get_object_detection(
            object_name, self.node.detection_handler_client
        )
        if detection is None or detection.xmin >= detection.xmax:
            return None, 0.0, 0.0

        tf_matrix = self._get_tf_matrix(
            "zed_left_camera_optical_frame", "base_link"
        )

        dense_msg = make_dense_cluster(
            self._latest_depth,
            self._camera_info,
            detection.xmin,
            detection.ymin,
            detection.xmax,
            detection.ymax,
            tf_matrix=tf_matrix,
            table_z=table_z,
        )
        if dense_msg is None:
            return None, 0.0, 0.0

        # Read Z from raw bytes directly (all floats, stride=12, Z at offset 8)
        data = np.frombuffer(dense_msg.data, dtype=np.float32)
        n_pts = len(data) // 3
        if n_pts == 0:
            return None, 0.0, 0.0
        zs = data[2::3]  # every 3rd float starting at index 2 = Z
        min_z = float(zs.min())
        height = float(zs.max()) - min_z

        self.node.get_logger().info(
            f"Dense cloud: {n_pts} pts (height={height:.2f} m)"
        )
        return dense_msg, min_z, height

    def flat_grasp_callback(self, msg):
        self.latest_flat_grasp = msg
        if self._collecting_samples:
            self._grasp_samples.append(msg)

    def execute(
        self, object_name: str, point: PointStamped, pick_params
    ) -> Tuple[bool, PickResult]:
        self.node.get_logger().info("Executing Pick Task")
        self.node.get_logger().info("Setting initial joint positions")

        is_flat_object = is_cutlery(object_name)

        if not pick_params.in_configuration:
            stare_position = "cutlery_stare" if is_flat_object else "table_stare"
            send_joint_goal(
                move_joints_action_client=self.node._move_joints_client,
                named_position=stare_position,
                velocity=0.75,
            )

        object_cluster = None
        height = 0.0

        if is_flat_object:
            self.node.get_logger().info(
                f"Cutlery detected: {object_name}. Waiting for flat_grasp_estimator..."
            )

            # Collect multiple poses and average for stable Z
            self._grasp_samples = []
            self._collecting_samples = True
            self.latest_flat_grasp = None

            # Wait for first pose (timeout if estimator isn't running)
            timeout_iterations = int(FLAT_GRASP_TIMEOUT / 0.1)
            for _ in range(timeout_iterations):
                if self.latest_flat_grasp is not None:
                    break
                time.sleep(0.1)

            if self.latest_flat_grasp is None:
                self._collecting_samples = False
                self.node.get_logger().error(
                    f"Timeout: flat_grasp_pose not received for {object_name}"
                )
                return False, None

            # Collect more samples over the sample window
            self.node.get_logger().info(
                f"First pose received, collecting {FLAT_GRASP_SAMPLES} samples..."
            )
            sample_iterations = int(FLAT_GRASP_SAMPLE_WINDOW / 0.1)
            for _ in range(sample_iterations):
                if len(self._grasp_samples) >= FLAT_GRASP_SAMPLES:
                    break
                time.sleep(0.1)

            self._collecting_samples = False
            samples = self._grasp_samples

            self.node.get_logger().info(f"Collected {len(samples)} grasp samples")

            if len(samples) == 0:
                self.node.get_logger().error("No grasp samples collected")
                return False, None

            # Average XY and Z across all samples for stability
            avg_x = np.median([s.pose.position.x for s in samples])
            avg_y = np.median([s.pose.position.y for s in samples])
            avg_z = np.median([s.pose.position.z for s in samples])
            z_std = np.std([s.pose.position.z for s in samples])

            self.node.get_logger().info(
                f"Averaged pose: X={avg_x:.3f}, Y={avg_y:.3f}, "
                f"Z={avg_z:.4f} (std={z_std:.4f}), +tweak={FLAT_GRASP_Z_TWEAK}"
            )

            # Use the last sample's orientation (PCA-computed) with averaged position
            grasp_pose = copy.deepcopy(samples[-1])
            grasp_pose.pose.position.x = float(avg_x)
            grasp_pose.pose.position.y = float(avg_y)
            grasp_pose.pose.position.z = float(avg_z) + FLAT_GRASP_Z_TWEAK

            # Do NOT call get_object_cluster — it adds the table as a
            # collision object which makes MoveIt reject all near-table paths
            object_cluster = None
            points = []

        else:
            # --- Perception cluster first (collision objects + table reference) ---
            for i in range(3):
                if point is not None and (
                    point.point.x != 0 and point.point.y != 0 and point.point.z != 0
                ):
                    self.node.get_logger().info(f"Going for point: {point}")
                elif object_name is not None and object_name != "":
                    self.node.get_logger().info(f"Going for object name: {object_name}")
                    point = get_object_point(
                        object_name, self.node.detection_handler_client
                    )
                    min_distance = pick_params.min_distance or 0.0
                    max_distance = pick_params.max_distance or PICK_MAX_DISTANCE
                    if not point_in_range(point, min_distance, max_distance):
                        self.node.get_logger().error(
                            f"Object {object_name} is out of range (distance: {(point.point.x**2 + point.point.y**2 + point.point.z**2) ** 0.5:.2f} m, expected range: [{min_distance}, {max_distance}] m)"
                        )
                        return False, None
                    if point.header.frame_id == "":
                        self.node.get_logger().error(f"Object {object_name} not found")
                        return False, None
                    if point.point.z > 1.0:
                        self.node.get_logger().error(f"Object {object_name} is too far")
                        return False, None
                else:
                    self.node.get_logger().error("No object name or point provided")
                    return False, None

                self.node.get_logger().info(f"Object in point: {point}")

                object_cluster = get_object_cluster(
                    point, self.node.pick_perception_3d_client
                )
                if object_cluster is not None:
                    self.node.get_logger().info("Object cluster detected")
                    break

            # Get table_z from cluster (min Z = table surface)
            cluster_min_z = None
            if object_cluster is not None:
                cloud_gen = point_cloud2.read_points(
                    object_cluster, field_names=("x", "y", "z"), skip_nans=True
                )
                pts_list = [list(p) for p in cloud_gen]
                self.node.get_logger().info(
                    f"Cluster has {len(pts_list)} points for table ref"
                )
                if pts_list:
                    pts_np = np.array(pts_list)
                    cluster_min_z = float(np.min(pts_np[:, 2]))
                    self.node.get_logger().info(
                        f"Table reference: min_z={cluster_min_z:.4f}, "
                        f"max_z={float(np.max(pts_np[:, 2])):.4f}"
                    )

            # --- Dense cloud from depth crop (primary method) ---
            dense_cloud = None
            if object_name is not None and object_name != "":
                dense_cloud, dense_min_z, dense_height = self._try_dense_cloud(
                    object_name, table_z=cluster_min_z
                )

            if object_cluster is None and dense_cloud is None:
                self.node.get_logger().error("No object cluster detected")
                return False, None

            # Use dense cloud for GPD if available, otherwise fall back to cluster
            if dense_cloud is not None:
                gpd_cloud = dense_cloud
                min_z = dense_min_z
                height = dense_height
                self.node.get_logger().info(
                    "Using dense depth-crop cloud for GPD"
                )
            else:
                gpd_cloud = object_cluster
                min_z = cluster_min_z or 0.0
                height = float(np.max(pts_np[:, 2])) - min_z if pts_list else 0.0
                self.node.get_logger().info(
                    "Using perception cluster for GPD (dense cloud unavailable)"
                )

            self.node.get_logger().info(f"Object cluster height: {height:.2f} m")

            # Symmetry completion (helps both dense and sparse clouds)
            self._debug_cluster_pub.publish(gpd_cloud)
            tf_cam = self._get_tf_matrix(
                "zed_left_camera_optical_frame", "base_link"
            )
            cam_pos = tf_cam[:3, 3] if tf_cam is not None else None
            object_cluster = complete_cluster(
                gpd_cloud,
                enable=True,
                plane_z=min_z,
                camera_position=cam_pos,
                logger=self.node.get_logger(),
            )
            self._debug_completed_pub.publish(object_cluster)

        # open gripper
        gripper_request = SetBool.Request()
        gripper_request.data = True
        self.node.get_logger().info("Open gripper")
        future = self.node._gripper_set_state_client.call_async(gripper_request)
        future = wait_for_future(future)
        result = future.result()
        self.node.get_logger().info(f"2 Gripper Result: {str(gripper_request.data)}")

        pick_result_success = False
        print("Gripper Result:", result)

        if is_flat_object:
            # Send the estimator pose + a 90° rotated alternative
            grasp_pose_alt = copy.deepcopy(grasp_pose)
            q_orig = R.from_quat(
                [
                    grasp_pose.pose.orientation.x,
                    grasp_pose.pose.orientation.y,
                    grasp_pose.pose.orientation.z,
                    grasp_pose.pose.orientation.w,
                ]
            )
            q_rotated = (q_orig * R.from_euler("z", 90, degrees=True)).as_quat()
            grasp_pose_alt.pose.orientation.x = q_rotated[0]
            grasp_pose_alt.pose.orientation.y = q_rotated[1]
            grasp_pose_alt.pose.orientation.z = q_rotated[2]
            grasp_pose_alt.pose.orientation.w = q_rotated[3]

            goal_msg = PickMotion.Goal()
            goal_msg.grasping_poses = [grasp_pose, grasp_pose_alt]
            goal_msg.grasping_scores = [1.0, 0.9]
            goal_msg.object_name = object_name

            self.node.get_logger().info(
                "Sending Pick Motion goal (flat grasp estimator)..."
            )
            future = self.node._pick_motion_action_client.send_goal_async(goal_msg)
            future = wait_for_future(future, timeout=30)

            if future:
                pick_result = future.result().get_result().result
                self.node.get_logger().info(f"Pick Motion Result: {pick_result}")
                if pick_result.success != 0:
                    pick_result_success = True

        else:
            # --- Attempt 0: Pre-computed grasps from database ---
            if object_name and self.grasp_db.has_object(object_name):
                t0 = time.time()
                db_grasps, db_scores = self.grasp_db.get_grasps(
                    object_name, point, min_z
                )
                db_time = (time.time() - t0) * 1000
                self.node.get_logger().info(
                    f"Database grasp attempt: object={object_name}, "
                    f"grasps_found={len(db_grasps)}, time={db_time:.0f}ms"
                )

                if db_grasps:
                    goal_msg = PickMotion.Goal()
                    goal_msg.grasping_poses = db_grasps[:10]
                    goal_msg.grasping_scores = db_scores[:10]

                    self.node.get_logger().info(
                        f"Sending {len(db_grasps[:10])} database grasps to pick motion..."
                    )
                    future = self.node._pick_motion_action_client.send_goal_async(goal_msg)
                    future = wait_for_future(future, timeout=30)
                    if future:
                        pick_result = future.result().get_result().result
                        if pick_result.success != 0:
                            self.node.get_logger().info(
                                f"Database grasp attempt: object={object_name}, success=True"
                            )
                            pick_result_success = True

                    if pick_result_success:
                        # Skip GPD fallback — database worked
                        pass
                    else:
                        self.node.get_logger().info(
                            f"Database grasp attempt: object={object_name}, success=False, "
                            f"falling back to GPD"
                        )

            # --- Fallback: GPD live ---
            if not pick_result_success:
                for CFG_PATH in CFG_PATHS:
                    cfg_path = CFG_PATH[0]
                    is_reversible = CFG_PATH[1]
                    if is_reversible and height < 0.06:
                        self.node.get_logger().warn(
                            "Object is too small for reversible grasping, skipping reversible grasps"
                        )
                        continue

                    grasp_poses, grasp_scores = get_grasps(
                        self.node.grasp_detection_client, object_cluster, cfg_path
                    )
                    if len(grasp_poses) == 0:
                        continue

                    if len(grasp_poses) > 10:
                        grasp_poses = grasp_poses[:10]
                        grasp_scores = grasp_scores[:10]

                    new_grasp_poses = []
                    new_grasp_scores = []
                    if is_reversible:
                        for pose, grasp_score in zip(grasp_poses, grasp_scores):
                            new_grasp_poses.append(pose)
                            new_grasp_scores.append(grasp_score)
                            reversed_pose = copy.deepcopy(pose)
                            q_orig = [
                                pose.pose.orientation.x,
                                pose.pose.orientation.y,
                                pose.pose.orientation.z,
                                pose.pose.orientation.w,
                            ]
                            q_orig_rot = R.from_quat(q_orig)
                            q_z_180_local = R.from_euler("z", 180, degrees=True)
                            q_result = (q_orig_rot * q_z_180_local).as_quat()
                            reversed_pose.pose.orientation.x = q_result[0]
                            reversed_pose.pose.orientation.y = q_result[1]
                            reversed_pose.pose.orientation.z = q_result[2]
                            reversed_pose.pose.orientation.w = q_result[3]
                            new_grasp_poses.append(reversed_pose)
                            new_grasp_scores.append(grasp_score)
                        new_grasp_scores = new_grasp_scores[:10]
                        new_grasp_poses = new_grasp_poses[:10]
                    else:
                        new_grasp_poses = grasp_poses
                        new_grasp_scores = grasp_scores

                    if len(new_grasp_poses) == 0:
                        self.node.get_logger().error("No grasp poses detected")
                        continue

                    goal_msg = PickMotion.Goal()
                    goal_msg.grasping_poses = new_grasp_poses
                    goal_msg.grasping_scores = new_grasp_scores

                    self.node.get_logger().info("Sending pick motion goal...")
                    future = self.node._pick_motion_action_client.send_goal_async(goal_msg)
                    future = wait_for_future(future, timeout=30)
                    if not future:
                        break
                    pick_result = future.result().get_result().result
                    self.node.get_logger().info(f"Pick Motion Result: {pick_result}")
                    if pick_result.success != 0:
                        pick_result_success = True
                        break
                    else:
                        time.sleep(0.2)

        if not pick_result_success:
            self.node.get_logger().error("Pick motion failed")
            return False, None

        # close gripper
        gripper_request = SetBool.Request()
        gripper_request.data = False
        self.node.get_logger().info("Closing gripper")
        future = self.node._gripper_set_state_client.call_async(gripper_request)
        future = wait_for_future(future)
        result = future.result()
        self.node.get_logger().info(f"Gripper Result: {str(gripper_request.data)}")

        self.node.get_logger().info("Returning to position")

        self.node.clear_octomap()
        for i in range(5):
            return_result = send_joint_goal(
                move_joints_action_client=self.node._move_joints_client,
                named_position="table_stare",
                velocity=0.5,
            )
            if return_result:
                break

        self.node.get_logger().info("Pick Task completed successfully")
        result.success = True
        return result.success, pick_result.pick_result

    def get_object_point(self, object_name: str) -> PointStamped:
        request = DetectionHandler.Request()
        request.label = object_name
        request.closest_object = False
        print("Request:", request)
        print("waiting for service")
        self.node.detection_handler_client.wait_for_service()
        future = self.node.detection_handler_client.call_async(request)
        print("waiting for future on detection_handler")
        future = wait_for_future(future)

        point = PointStamped()

        if len(future.result().detection_array.detections) == 1:
            self.node.get_logger().info(f"Object {object_name} found")
            point = future.result().detection_array.detections[0].point3d
        elif len(future.result().detection_array.detections) > 1:
            self.node.get_logger().info(
                "Multiple objects found, selecting the closest one"
            )
            closest_object = future.result().detection_array.detections[0]
            closest_distance = float("inf")
            for detection in future.result().detection_array.detections:
                distance = (
                    detection.point3d.point.x**2
                    + detection.point3d.point.y**2
                    + detection.point3d.point.z**2
                ) ** 0.5
                if distance < closest_distance:
                    closest_distance = distance
                    closest_object = detection
            point = closest_object.point3d
        else:
            self.node.get_logger().error(f"Object {object_name} not found")

        self.node.get_logger().info(f"Object {object_name} found at: {point}")
        return point

    def get_object_cluster(self, point: PointStamped):
        request = PickPerceptionService.Request()
        request.point = point
        request.add_collision_objects = True
        self.node.pick_perception_3d_client.wait_for_service()
        future = self.node.pick_perception_3d_client.call_async(request)
        future = wait_for_future(future, timeout=10)

        if future is None:
            self.node.get_logger().error("Pick Perception Service call failed")
            return None

        pcl_result = future.result().cluster_result
        if len(pcl_result.data) == 0:
            self.node.get_logger().error("No object cluster detected")
            return None
        self.node.get_logger().info(
            f"Object cluster detected: {len(pcl_result.data)} points"
        )
        return pcl_result
