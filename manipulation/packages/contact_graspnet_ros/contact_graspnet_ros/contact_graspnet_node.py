#!/usr/bin/env python3

import os
import sys
import contextlib
import numpy as np
import rclpy
import tf2_ros
import torch
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from frida_interfaces.srv import GraspDetection
from frida_constants.manipulation_constants import GRASP_DETECTION_SERVICE
import sensor_msgs_py.point_cloud2 as pc2

# sys.path must be extended before importing contact_graspnet_pytorch.
# realpath resolves the build-dir symlink back to the source tree so the
# sibling contact_graspnet package is found regardless of CWD.
package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
submodule_path = os.path.join(os.path.dirname(package_path), "contact_graspnet")
sys.path.append(submodule_path)

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator  # noqa: E402
from contact_graspnet_pytorch import config_utils  # noqa: E402


class ContactGraspNetNode(Node):
    def __init__(self):
        super().__init__("contact_graspnet_node")

        self.declare_parameter("ckpt_dir", "checkpoints/contact_graspnet")
        self.declare_parameter("forward_passes", 1)
        self.declare_parameter("z_range", [0.2, 1.8])
        self.declare_parameter("pick_min_height", 0.03)
        # FP16 autocast for the forward pass. Jetson Orin's tensor cores only
        # accelerate FP16/BF16 — pure FP32 leaves most of the GPU idle. Expect
        # 1.5–2× inference speedup. Toggle off if you suspect accuracy drift.
        self.declare_parameter("use_fp16", True)
        self.use_fp16 = (
            self.get_parameter("use_fp16").get_parameter_value().bool_value
            and torch.cuda.is_available()
        )

        ckpt_dir = self.get_parameter("ckpt_dir").get_parameter_value().string_value
        if not os.path.isabs(ckpt_dir):
            ckpt_dir = os.path.join(submodule_path, ckpt_dir)

        forward_passes = (
            self.get_parameter("forward_passes").get_parameter_value().integer_value
        )

        self.get_logger().info(f"Loading Contact-GraspNet model from {ckpt_dir}...")

        try:
            self.global_config = config_utils.load_config(
                ckpt_dir, batch_size=forward_passes
            )
            self.grasp_estimator = GraspEstimator(self.global_config)

            from contact_graspnet_pytorch.checkpoints import CheckpointIO

            model_checkpoint_dir = os.path.join(ckpt_dir, "checkpoints")
            checkpoint_io = CheckpointIO(
                checkpoint_dir=model_checkpoint_dir, model=self.grasp_estimator.model
            )
            checkpoint_io.load("model.pt")

            self.get_logger().info(
                f"Model loaded (fp16={self.use_fp16}). Running GPU warm-up pass..."
            )
            # Warm-up: one dummy forward pass so the first real pick avoids
            # CUDA JIT compilation overhead (~10 s cold-start on Jetson Orin).
            # Run under the same autocast context as real inference so the JIT
            # caches FP16 kernels, not FP32 ones.
            dummy_pc = np.random.rand(50, 3).astype(np.float32)
            with self._inference_context():
                self.grasp_estimator.predict_scene_grasps(
                    dummy_pc, local_regions=False, filter_grasps=False, forward_passes=1
                )
            self.get_logger().info("Warm-up complete — node ready for grasp requests.")

        except Exception as e:
            import traceback

            self.get_logger().error(
                f"Failed to load model: {str(e)}\n{traceback.format_exc()}"
            )

        self.srv = self.create_service(
            GraspDetection, GRASP_DETECTION_SERVICE, self.detect_grasps_callback
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _inference_context(self):
        """Autocast context used for both warm-up and real inference.

        FP16 only applies to the model's matmul/conv layers — PointNet++ ops
        (FPS, ball-query, etc.) stay in FP32 because they're hand-written CUDA
        kernels that don't go through the autocast dispatch. That's fine: those
        ops aren't the bottleneck.
        """
        if self.use_fp16:
            return torch.cuda.amp.autocast(dtype=torch.float16)
        return contextlib.nullcontext()

    # ──────────────────────────────────────────────────────────────────────────
    # Service callback
    # ──────────────────────────────────────────────────────────────────────────

    def detect_grasps_callback(self, request, response):
        self.get_logger().info("Received grasp detection request")

        if not hasattr(self, "grasp_estimator"):
            response.success = False
            response.message = "Model not loaded"
            return response

        # ── 1. Convert PointCloud2 → numpy ───────────────────────────────────
        try:
            points_list = []
            for p in pc2.read_points(
                request.input_cloud, field_names=("x", "y", "z"), skip_nans=True
            ):
                points_list.append([p[0], p[1], p[2]])

            pc_full = np.array(points_list, dtype=np.float32)

            if pc_full.shape[0] == 0:
                response.success = False
                response.message = "Empty point cloud"
                return response

            self.get_logger().info(
                f"Point cloud: {pc_full.shape[0]} points in frame "
                f"'{request.input_cloud.header.frame_id}'"
            )

        except Exception as e:
            self.get_logger().error(f"Cloud conversion error: {e}")
            response.success = False
            response.message = f"Cloud conversion error: {e}"
            return response

        # ── 2. Run inference ──────────────────────────────────────────────────
        try:
            forward_passes = (
                self.get_parameter("forward_passes").get_parameter_value().integer_value
            )

            with self._inference_context():
                pred_grasps_cam, scores, contact_pts, _ = (
                    self.grasp_estimator.predict_scene_grasps(
                        pc_full,
                        local_regions=False,
                        filter_grasps=False,
                        forward_passes=forward_passes,
                    )
                )

            grasps = pred_grasps_cam[-1]
            grasp_scores = scores[-1]

            self.get_logger().info(f"Raw CGN output: {len(grasps)} grasp candidates")

            # ── 3. Sort by score descending ───────────────────────────────────
            idx = np.argsort(grasp_scores)[::-1]
            grasps = grasps[idx]
            grasp_scores = grasp_scores[idx]

            # ── 4. Transform to base_link FIRST ───────────────────────────────
            # All subsequent filters operate in the world frame (Z points up).
            # The camera is mounted at an angle — filtering in camera frame causes
            # "downward in camera" to be lateral/inclined in base_link.
            source_frame = request.input_cloud.header.frame_id
            output_frame = source_frame
            T_base_cloud = np.eye(4, dtype=np.float64)

            if source_frame and source_frame != "base_link":
                try:
                    tf_stamped = self.tf_buffer.lookup_transform(
                        "base_link",
                        source_frame,
                        request.input_cloud.header.stamp,
                        timeout=Duration(seconds=1.0),
                    )
                    T_base_cloud = self._transform_to_matrix(tf_stamped)
                    output_frame = "base_link"
                    self.get_logger().info(f"TF: '{source_frame}' → base_link OK")
                except Exception as tf_ex:
                    self.get_logger().warn(
                        f"TF '{source_frame}'→base_link failed: {tf_ex}; "
                        "outputting in source frame"
                    )

            grasps = np.array([T_base_cloud @ g for g in grasps])

            # Also transform point cloud for cluster_min_z in base_link.
            ones = np.ones((pc_full.shape[0], 1), dtype=np.float64)
            pc_base = (T_base_cloud @ np.hstack([pc_full.astype(np.float64), ones]).T).T
            cluster_min_z = float(np.min(pc_base[:, 2]))
            cluster_max_z = float(np.max(pc_base[:, 2]))
            self.get_logger().info(
                f"Cluster in base_link: z=[{cluster_min_z:.3f}, {cluster_max_z:.3f}]"
            )

            # ── 5. Approach filter in base_link ───────────────────────────────
            # g[2,2] = Z component of approach axis in base_link.
            #   -1 = straight down (ideal top-down grasp)
            #    0 = horizontal (side grasp — rejected)
            #   +1 = straight up  (upward — rejected)
            # Threshold -0.3 ≈ 17° below horizontal.
            PREFER_TOPDOWN = -0.3
            valid = np.array([g[2, 2] < PREFER_TOPDOWN for g in grasps])

            if valid.sum() == 0:
                self.get_logger().warn(
                    "No top-down grasps (g[2,2]<-0.3) — relaxing to any downward approach"
                )
                valid = np.array([g[2, 2] < 0.0 for g in grasps])

            if valid.sum() == 0:
                self.get_logger().warn(
                    "No downward grasps at all — using unfiltered set"
                )
            else:
                grasps = grasps[valid]
                grasp_scores = grasp_scores[valid]

            self.get_logger().info(
                f"Approach filter: {int(valid.sum())} grasps with downward approach"
            )

            # ── 6. Roll normalisation + snap-to-vertical in base_link ─────────
            # world_up=[0,0,1] is correct here because we are in base_link.
            # CGN sometimes predicts a few-degree tilt on near-vertical grasps
            # which makes the gripper arrive slightly inclined and miss.
            normalised = []
            world_up = np.array([0.0, 0.0, 1.0])
            SNAP_VERTICAL = 0.92  # cos(≈23°)

            for g in grasps:
                z = g[:3, 2].copy()
                z /= np.linalg.norm(z)

                if abs(z[2]) >= SNAP_VERTICAL:
                    z = np.array([0.0, 0.0, float(np.sign(z[2]))])

                ref = world_up
                perp = ref - np.dot(ref, z) * z
                if np.linalg.norm(perp) < 0.15:
                    ref = np.array([1.0, 0.0, 0.0])
                    perp = ref - np.dot(ref, z) * z
                y_new = perp / np.linalg.norm(perp)
                x_new = np.cross(y_new, z)
                x_new /= np.linalg.norm(x_new)
                m = g.copy()
                m[:3, 0] = x_new
                m[:3, 1] = y_new
                m[:3, 2] = z
                normalised.append(m)

            grasps = np.array(normalised)

            # ── 7. Height filter ──────────────────────────────────────────────
            # Drop grasps too close to the table surface — MoveIt struggles to
            # plan near the table collision box.
            pick_min_height = (
                self.get_parameter("pick_min_height").get_parameter_value().double_value
            )
            min_z_threshold = cluster_min_z + pick_min_height
            valid_height = np.array([g[2, 3] >= min_z_threshold for g in grasps])

            if valid_height.sum() > 0:
                grasps = grasps[valid_height]
                grasp_scores = grasp_scores[valid_height]
                self.get_logger().info(
                    f"Height filter: {int(valid_height.sum())}/{len(valid_height)} grasps "
                    f"above z={min_z_threshold:.3f} m"
                )
            else:
                self.get_logger().warn(
                    "All grasps near table surface — skipping height filter"
                )

            self.get_logger().info(
                f"Final: {len(grasps)} grasps. "
                f"Top score={float(grasp_scores[0]):.3f} "
                f"z={float(grasps[0][2,3]):.3f} m"
            )

            # ── 8. Build response ─────────────────────────────────────────────
            response.grasp_poses = []
            response.grasp_scores = []

            for i in range(len(grasps)):
                matrix = grasps[i]
                score = float(grasp_scores[i])

                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = request.input_cloud.header.stamp
                pose_stamped.header.frame_id = output_frame

                pose_stamped.pose.position.x = float(matrix[0, 3])
                pose_stamped.pose.position.y = float(matrix[1, 3])
                pose_stamped.pose.position.z = float(matrix[2, 3])

                q = self._matrix_to_quaternion(matrix[:3, :3])
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]

                response.grasp_poses.append(pose_stamped)
                response.grasp_scores.append(score)

            response.success = True
            response.message = f"Detected {len(grasps)} grasps"

        except Exception as e:
            import traceback as tb

            self.get_logger().error(f"Inference error: {e}\n{tb.format_exc()}")
            response.success = False
            response.message = f"Inference error: {e}"

        return response

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _transform_to_matrix(self, tf_stamped) -> np.ndarray:
        t = tf_stamped.transform.translation
        r = tf_stamped.transform.rotation
        qx, qy, qz, qw = r.x, r.y, r.z, r.w
        R = np.array(
            [
                [
                    1 - 2 * (qy**2 + qz**2),
                    2 * (qx * qy - qz * qw),
                    2 * (qx * qz + qy * qw),
                ],
                [
                    2 * (qx * qy + qz * qw),
                    1 - 2 * (qx**2 + qz**2),
                    2 * (qy * qz - qx * qw),
                ],
                [
                    2 * (qx * qz - qy * qw),
                    2 * (qy * qz + qx * qw),
                    1 - 2 * (qx**2 + qy**2),
                ],
            ]
        )
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _matrix_to_quaternion(self, m: np.ndarray):
        tr = np.trace(m)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * s
            qx = (m[2, 1] - m[1, 2]) / s
            qy = (m[0, 2] - m[2, 0]) / s
            qz = (m[1, 0] - m[0, 1]) / s
        elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            qw = (m[2, 1] - m[1, 2]) / s
            qx = 0.25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = 0.25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = 0.25 * s
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = ContactGraspNetNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
