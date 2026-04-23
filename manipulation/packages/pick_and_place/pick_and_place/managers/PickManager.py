from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import PickPerceptionService, DetectionHandler
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool
from pick_and_place.utils.grasp_utils import get_grasps
from pick_and_place.utils.perception_utils import get_object_cluster, point_in_range
from frida_interfaces.action import PickMotion
from frida_interfaces.msg import PickResult
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)
from frida_constants.manipulation_constants import PICK_MAX_DISTANCE
from typing import Tuple
import copy
from scipy.spatial.transform import Rotation as R
from sensor_msgs_py import point_cloud2
import numpy as np
from pick_and_place.utils.perception_utils import get_object_point


class PickManager:
    node = None

    def __init__(self, node):
        self.node = node

    def execute(
        self, object_name: str, point: PointStamped, pick_params
    ) -> Tuple[bool, PickResult]:
        self.node.get_logger().info("Executing Pick Task")
        self.node.get_logger().info("Setting initial joint positions")

        # time.sleep(10)
        # Set initial joint positions
        if not pick_params.in_configuration:
            send_joint_goal(
                move_joints_action_client=self.node._move_joints_client,
                named_position="table_stare",
                velocity=0.75,
            )

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
                    self.node.get_logger().error(
                        f"Object {object_name} not found, please provide a point"
                    )
                    return False, None
                if point.point.z > 1.0:
                    self.node.get_logger().error(f"Object {object_name} is too far")
                    return False, None
            else:
                self.node.get_logger().error("No object name or point provided")
                return False, None

            self.node.get_logger().info(f"Object in point: {point}")

            # Call Perception Service to get object cluster and generate collision objects
            object_cluster = get_object_cluster(
                point, self.node.pick_perception_3d_client
            )
            if object_cluster is not None:
                self.node.get_logger().info("Object cluster detected")
                break

        if object_cluster is None:
            self.node.get_logger().error("No object cluster detected")
            return False, None

        # 8. Compute Centroid/Height and Set Pour Pose
        cloud_gen = point_cloud2.read_points(
            object_cluster, field_names=("x", "y", "z"), skip_nans=True
        )
        points = [list(p) for p in cloud_gen]
        if not points:
            self.node.get_logger().error("Empty bowl cluster")
            return False, None

        points_np = np.array(points)
        max_z = np.max(points_np[:, 2])
        min_z = np.min(points_np[:, 2])
        height = max_z - min_z
        self.node.get_logger().info(f"Object cluster height: {height:.2f} m")
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

        # Call AnyGrasp for grasp detection (single call, persistent service)
        grasp_poses, grasp_scores = get_grasps(
            self.node.grasp_detection_client, object_cluster
        )

        if len(grasp_poses) == 0:
            self.node.get_logger().error("No grasp poses detected")
        else:
            self.node.get_logger().info(f"Detected {len(grasp_poses)} grasps, scores: {grasp_scores[:5]}")

            # Take top 5 grasps (already sorted by score from AnyGrasp NMS)
            grasp_poses = grasp_poses[:5]
            grasp_scores = grasp_scores[:5]

            # For tall objects (>=6cm), also try each grasp rotated 180° around Z
            # to handle symmetric approach directions
            if height >= 0.06:
                augmented_poses = []
                augmented_scores = []
                for pose, score in zip(grasp_poses, grasp_scores):
                    augmented_poses.append(pose)
                    augmented_scores.append(score)
                    reversed_pose = copy.deepcopy(pose)
                    q_orig = [
                        pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w,
                    ]
                    q_result = (R.from_quat(q_orig) * R.from_euler("z", 180, degrees=True)).as_quat()
                    reversed_pose.pose.orientation.x = q_result[0]
                    reversed_pose.pose.orientation.y = q_result[1]
                    reversed_pose.pose.orientation.z = q_result[2]
                    reversed_pose.pose.orientation.w = q_result[3]
                    augmented_poses.append(reversed_pose)
                    augmented_scores.append(score)
                grasp_poses = augmented_poses[:5]
                grasp_scores = augmented_scores[:5]

            self.node.get_logger().info(
                f"Sending {len(grasp_poses)} grasp candidates to pick server"
            )

            goal_msg = PickMotion.Goal()
            goal_msg.grasping_poses = grasp_poses
            goal_msg.grasping_scores = grasp_scores

            self.node.get_logger().info("Sending pick motion goal...")
            future = self.node._pick_motion_action_client.send_goal_async(goal_msg)
            future = wait_for_future(future, timeout=30)
            if future:
                pick_result = future.result().get_result().result
                self.node.get_logger().info(f"Pick Motion Result: {pick_result}")
                if pick_result.success != 0:
                    pick_result_success = True

        if not pick_result_success:
            self.node.get_logger().error("Pick motion failed")
            return False, None

        # close gripper
        for i in range(2):
            gripper_request = SetBool.Request()
            gripper_request.data = False
            self.node.get_logger().info("Closing gripper")
            future = self.node._gripper_set_state_client.call_async(gripper_request)
            future = wait_for_future(future)
            result = future.result()
            self.node.get_logger().info(
                f"1 Gripper Result: {str(gripper_request.data)}"
            )

        self.node.get_logger().info("Returning to position")

        self.node.clear_octomap()
        for i in range(5):
            # return to configured position
            return_result = send_joint_goal(
                move_joints_action_client=self.node._move_joints_client,
                named_position="table_stare",
                velocity=0.5,
            )
            if return_result:
                break
        # self.node.get_logger().info("Waiting for 10 seconds")
        # time.sleep(10)
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
