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
import time
import copy
from scipy.spatial.transform import Rotation as R
from sensor_msgs_py import point_cloud2
import numpy as np
from pick_and_place.utils.perception_utils import get_object_point

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
        for CFG_PATH in CFG_PATHS:
            cfg_path = CFG_PATH[0]
            is_reversible = CFG_PATH[1]
            if is_reversible and height < 0.06:
                self.node.get_logger().warn(
                    "Object is too small for reversible grasping, skipping reversible grasps"
                )
                continue
            self.node.get_logger().info(f"CFG_PATH: {CFG_PATH}")
            # Call Grasp Pose Detection
            grasp_poses, grasp_scores = get_grasps(
                self.node.grasp_detection_client, object_cluster, cfg_path
            )
            if len(grasp_poses) == 0:
                self.node.get_logger().error(
                    f"No grasp poses detected with cfg {cfg_path}"
                )
                continue

            self.node.get_logger().info(f"Detected grasps with scores: {grasp_scores}")

            # Sort grasp poses by score in descending order (highest scores first)
            # grasp_poses, grasp_scores = zip(
            #     *sorted(
            #         zip(grasp_poses, grasp_scores),
            #         key=lambda x: x[1],
            #         reverse=True,
            #     )
            # )
            # grasp_poses, grasp_scores = grasp_poses[:5], grasp_scores[:5]

            # randomly pick 5 grasps
            if len(grasp_poses) > 5:
                indices = np.random.choice(len(grasp_poses), size=5, replace=False)
                grasp_poses = [grasp_poses[i] for i in indices]
                grasp_scores = [grasp_scores[i] for i in indices]
            else:
                # if less than 5 grasps, just take them all
                grasp_poses = grasp_poses[:5]
                grasp_scores = grasp_scores[:5]

            self.node.get_logger().info(
                f"Top 5 grasp poses detected with scores: {grasp_scores}"
            )

            # reverse grasps (turn 180 degrees in z)
            new_grasp_poses = []
            new_grasp_scores = []
            if is_reversible:
                for pose, grasp_score in zip(grasp_poses, grasp_scores):
                    new_grasp_poses.append(pose)
                    new_grasp_scores.append(grasp_score)
                    # Reverse the pose (turn 180 degrees in z)
                    reversed_pose = copy.deepcopy(pose)
                    # 180 degrees rotation around Z axis
                    # Rotate 180 degrees around the local Z axis (end-effector frame)
                    q_orig = [
                        pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w,
                    ]
                    q_orig_rot = R.from_quat(q_orig)
                    q_z_180_local = R.from_euler("z", 180, degrees=True)
                    q_result = (q_orig_rot * q_z_180_local).as_quat()  # [x, y, z, w]
                    reversed_pose.pose.orientation.x = q_result[0]
                    reversed_pose.pose.orientation.y = q_result[1]
                    reversed_pose.pose.orientation.z = q_result[2]
                    reversed_pose.pose.orientation.w = q_result[3]
                    new_grasp_poses.append(reversed_pose)
                    new_grasp_scores.append(grasp_score)
                new_grasp_scores = new_grasp_scores[:5]
                new_grasp_poses = new_grasp_poses[:5]
            else:
                new_grasp_poses = grasp_poses
                new_grasp_scores = grasp_scores

            if len(new_grasp_poses) == 0:
                self.node.get_logger().error("No grasp poses detected")
                continue

            # Call Pick Motion Action

            # Create goal
            goal_msg = PickMotion.Goal()
            goal_msg.grasping_poses = new_grasp_poses
            goal_msg.grasping_scores = new_grasp_scores

            # Send goal
            self.node.get_logger().info("Sending pick motion goal...")
            future = self.node._pick_motion_action_client.send_goal_async(goal_msg)
            future = wait_for_future(future, timeout=30)
            if not future:
                break
            # Check result
            pick_result = future.result().get_result().result
            self.node.get_logger().info(f"Pick Motion Result: {pick_result}")
            if pick_result.success != 0:
                pick_result_success = True
                break
            else:
                # give time for new gpd
                time.sleep(1.0)

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
