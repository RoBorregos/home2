#!/usr/bin/env python3

import os
import sys
import numpy as np
import rclpy
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

            for i in range(len(grasps)):
                matrix = grasps[i]  # 4x4 matrix
                score = grasp_scores[i]

                pose_stamped = PoseStamped()
                pose_stamped.header = request.input_cloud.header

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
