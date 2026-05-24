#!/usr/bin/env python3

import os
import sys
import numpy as np
import rclpy
import tf2_ros
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

        ckpt_dir = self.get_parameter("ckpt_dir").get_parameter_value().string_value
        # Resolve absolute path for ckpt_dir if relative
        if not os.path.isabs(ckpt_dir):
            ckpt_dir = os.path.join(submodule_path, ckpt_dir)

        forward_passes = (
            self.get_parameter("forward_passes").get_parameter_value().integer_value
        )

        self.get_logger().info(f"Loading model from {ckpt_dir}...")

        try:
            self.global_config = config_utils.load_config(
                ckpt_dir, batch_size=forward_passes
            )
            self.grasp_estimator = GraspEstimator(self.global_config)

            # Load weights manually as GraspEstimator doesn't do it in __init__
            from contact_graspnet_pytorch.checkpoints import CheckpointIO

            model_checkpoint_dir = os.path.join(ckpt_dir, "checkpoints")
            checkpoint_io = CheckpointIO(
                checkpoint_dir=model_checkpoint_dir, model=self.grasp_estimator.model
            )
            checkpoint_io.load("model.pt")

            self.get_logger().info("Contact-GraspNet model loaded successfully.")
        except Exception as e:
            import traceback

            self.get_logger().error(
                f"Failed to load model: {str(e)}\n{traceback.format_exc()}"
            )
            # We don't exit here to allow the node to stay alive for debugging

        self.srv = self.create_service(
            GraspDetection, GRASP_DETECTION_SERVICE, self.detect_grasps_callback
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def detect_grasps_callback(self, request, response):
        self.get_logger().info("Received grasp detection request")

        if not hasattr(self, "grasp_estimator"):
            response.success = False
            response.message = "Model not loaded"
            return response

        # 1. Convert PointCloud2 to numpy
        try:
            # Get points from PointCloud2
            points_list = []
            for p in pc2.read_points(
                request.input_cloud, field_names=("x", "y", "z"), skip_nans=True
            ):
                points_list.append([p[0], p[1], p[2]])

            pc_full = np.array(points_list).astype(np.float32)

            if pc_full.shape[0] == 0:
                response.success = False
                response.message = "Empty point cloud"
                return response

        except Exception as e:
            self.get_logger().error(f"Error converting cloud: {str(e)}")
            response.success = False
            response.message = f"Cloud conversion error: {str(e)}"
            return response

        # 2. Run Inference
        try:
            # We assume for now we don't have segmentation masks (local_regions=False)
            # or we process the whole scene
            forward_passes = (
                self.get_parameter("forward_passes").get_parameter_value().integer_value
            )

            self.get_logger().info(
                f"Processing cloud with {pc_full.shape[0]} points..."
            )

            # Predict
            pred_grasps_cam, scores, contact_pts, _ = (
                self.grasp_estimator.predict_scene_grasps(
                    pc_full,
                    local_regions=False,
                    filter_grasps=False,
                    forward_passes=forward_passes,
                )
            )

            # pred_grasps_cam is a dict if local_regions=True, otherwise it's a list or array
            # In our case with local_regions=False, it returns results for the whole scene (key -1)
            grasps = pred_grasps_cam[-1]
            grasp_scores = scores[-1]

            self.get_logger().info(f"Detected {len(grasps)} grasps.")

            # 3. Fill Response
            response.grasp_poses = []
            response.grasp_scores = []

            # Sort by score
            idx = np.argsort(grasp_scores)[::-1]
            grasps = grasps[idx]
            grasp_scores = grasp_scores[idx]

            # ── Post-processing ──────────────────────────────────────────
            # 1. Filter: drop grasps that approach from below the table.
            #    Column 2 of the 4x4 matrix is the approach direction in
            #    base_link frame.  Positive Z means the gripper would have
            #    to come from under the table — physically impossible.
            #    Allow up to 0.3 of upward tilt (≈17° tolerance) for
            #    slightly angled side approaches.
            MAX_UPWARD = 0.3
            valid = np.array([g[2, 2] < MAX_UPWARD for g in grasps])
            if valid.sum() == 0:
                self.get_logger().warn(
                    "All grasps approach from below — using unfiltered set"
                )
            else:
                grasps = grasps[valid]
                grasp_scores = grasp_scores[valid]

            # 2. Normalise roll: snap the gripper's Y-axis to be as close
            #    to world-up as possible, keeping the approach (Z) fixed.
            #    Additionally snap near-vertical approach axes to exact vertical:
            #    CGN sometimes predicts a 2-5° tilt on top-down grasps which
            #    causes the gripper to arrive slightly inclined and miss the object.
            normalised = []
            world_up = np.array([0.0, 0.0, 1.0])
            SNAP_VERTICAL = (
                0.92  # cos(≈23°) — snap to [0,0,±1] if approach is this close
            )
            for g in grasps:
                z = g[:3, 2].copy()
                z /= np.linalg.norm(z)

                # Snap near-vertical approaches to exactly vertical
                if abs(z[2]) >= SNAP_VERTICAL:
                    z = np.array([0.0, 0.0, float(np.sign(z[2]))])

                ref = world_up
                perp = ref - np.dot(ref, z) * z
                if np.linalg.norm(perp) < 0.15:  # near-vertical approach
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
            # ────────────────────────────────────────────────────────────

            self.get_logger().info(
                f"After filtering/normalisation: {len(grasps)} grasps remain."
            )

            # Transform grasps to base_link (matches GPD output convention).
            # CGN predicts in the input cloud frame; if the cluster was captured
            # in camera frame the positions would shift as the arm moves.
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
                    self.get_logger().info(
                        f"Transformed grasps from '{source_frame}' to base_link"
                    )
                except Exception as tf_ex:
                    self.get_logger().warn(
                        f"TF '{source_frame}'→base_link failed: {tf_ex}; "
                        "outputting in original frame"
                    )

            grasps = np.array([T_base_cloud @ g for g in grasps])

            # 3. Filter grasps too close to the cluster bottom (≈ table surface).
            #    The table collision box's padding forces MoveIt to plan wide arcs
            #    when the grasp target is near the table — drop these early.
            #    Cluster is in base_link so pc_full Z is height above the floor.
            cluster_min_z = float(np.min(pc_full[:, 2]))
            TABLE_MARGIN = 0.03  # 3 cm above cluster bottom
            valid_height = np.array(
                [g[2, 3] >= cluster_min_z + TABLE_MARGIN for g in grasps]
            )
            if valid_height.sum() > 0:
                grasps = grasps[valid_height]
                grasp_scores = grasp_scores[valid_height]
                self.get_logger().info(
                    f"Height filter: {valid_height.sum()}/{len(valid_height)} grasps "
                    f"above z={cluster_min_z + TABLE_MARGIN:.3f} m"
                )
            else:
                self.get_logger().warn(
                    "All grasps near table surface — skipping height filter"
                )

            for i in range(len(grasps)):
                matrix = grasps[i]  # 4x4 matrix
                score = grasp_scores[i]

                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = request.input_cloud.header.stamp
                pose_stamped.header.frame_id = output_frame

                # Extract position
                pose_stamped.pose.position.x = float(matrix[0, 3])
                pose_stamped.pose.position.y = float(matrix[1, 3])
                pose_stamped.pose.position.z = float(matrix[2, 3])

                # Extract orientation (Rotation matrix to Quaternion)
                q = self.matrix_to_quaternion(matrix[:3, :3])
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]

                response.grasp_poses.append(pose_stamped)
                response.grasp_scores.append(float(score))

            response.success = True
            response.message = f"Successfully detected {len(grasps)} grasps"

        except Exception as e:
            self.get_logger().error(f"Error during inference: {str(e)}")
            response.success = False
            response.message = f"Inference error: {str(e)}"

        return response

    def _transform_to_matrix(self, tf_stamped):
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

    def matrix_to_quaternion(self, m):
        # Simple matrix to quaternion conversion
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
