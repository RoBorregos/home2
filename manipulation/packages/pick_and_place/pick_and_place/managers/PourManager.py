import numpy as np
from tf2_ros import Buffer, TransformListener
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import (
    DetectionHandler,
    GetCollisionObjects,
    PickPerceptionService
)
from geometry_msgs.msg import PointStamped, PoseStamped
from std_srvs.srv import SetBool
from pick_and_place.utils.grasp_utils import get_grasps
from frida_interfaces.action import PickMotion, PourMotion
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
import time
from scipy.spatial.transform import Rotation as R

import copy

# path, is reversible
CFG_PATHS = [
    ["/workspace/src/home2/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper_testing.cfg", True],
]


class PourManager:
    node = None

    def __init__(self, node):
        self.node = node

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self.debug_object_point_pub = self.node.create_publisher(
            PointStamped,
            "/manipulation/debug_object_point",
            10
        )
        
        self.debug_bowl_point_pub = self.node.create_publisher(
            PointStamped,
            "/manipulation/debug_bowl_point",
            10
        )
        
        self.debug_centroid_pub = self.node.create_publisher(
            PointStamped,
            "/manipulation/debug_bowl_centroid",
            10
        )

    def execute(self, object_name: str, container_object_name: str) -> bool:
        self.node.get_logger().info("Executing Pour Task")

        # 1. Initial Position
        send_joint_goal(
            move_joints_action_client=self.node._move_joints_client,
            named_position="table_stare",
            velocity=0.3,
        )

        # 2. Get Object Point
        object_point = self.get_object_point(object_name)
        if object_point is None:
            self.node.get_logger().error(f"Object {object_name} not found")
            return False, None
        
        self.debug_object_point_pub.publish(object_point)

        # 3. Get Bowl Point
        bowl_point = self.get_object_point(container_object_name)
        if bowl_point is None:
            self.node.get_logger().error(f"Bowl {container_object_name} not found")
            return False, None

        self.debug_bowl_point_pub.publish(bowl_point)

        # 4. Get Clusters
        bowl_cluster = self.get_object_cluster(bowl_point, add_primitives=False)
        if bowl_cluster is None:
            return False, None
        self.node.remove_all_collision_object(attached=False)
        
        object_cluster = self.get_object_cluster(object_point, add_primitives=True)
        if object_cluster is None:
            return False, None

        # 5. Open Gripper
        gripper_request = SetBool.Request()
        gripper_request.data = True
        future = self.node._gripper_set_state_client.call_async(gripper_request)
        wait_for_future(future)

        # 8. Compute Centroid/Height and Set Pour Pose
        cloud_gen = point_cloud2.read_points(
            bowl_cluster, field_names=("x", "y", "z"), skip_nans=True
        )
        points = [list(p) for p in cloud_gen]
        if not points:
            self.node.get_logger().error("Empty bowl cluster")
            return False, None
        
        points_np = np.array(points)
        centroid = np.mean(points_np, axis=0)
        max_z = np.max(points_np[:, 2])
        self.node.get_logger().info(f"Bowl Centroid: {centroid}, Max Z: {max_z}")
        
        bowl_centroid_point = PointStamped()
        bowl_centroid_point.header.frame_id = bowl_cluster.header.frame_id
        bowl_centroid_point.header.stamp = self.node.get_clock().now().to_msg()
        bowl_centroid_point.point.x = float(centroid[0])
        bowl_centroid_point.point.y = float(centroid[1])
        bowl_centroid_point.point.z = float(max_z + 0.05)  # Slightly above the bowl
        self.debug_centroid_pub.publish(bowl_centroid_point)

        pick_result = None
        # 6. Pick Motion
        pick_success, pick_result = self.pick_motion(object_cluster)
        if not pick_success:
            self.node.get_logger().error("Pick motion failed")
            return False, pick_result.pick_result

        pour_pose = PoseStamped()
        pour_pose.header.frame_id = bowl_centroid_point.header.frame_id
        pour_pose.header.stamp = self.node.get_clock().now().to_msg()
        pour_pose.pose.position.x = float(centroid[0])
        pour_pose.pose.position.y = float(centroid[1])
        pour_pose.pose.position.z = float(max_z + 0.05)  # Slightly above the bowl
        pour_pose.pose.position.z += 0.05  # Slightly above the bowl

        if pick_result is not None:
           
            pick_pose = getattr(pick_result.pick_result, "pick_pose", None)
            if pick_pose:
                pour_pose.pose.orientation = pick_pose.pose.orientation
                self.node.get_logger().info(f"Pour Pose Orientation: {pour_pose.pose.orientation}")
        else:
            # self.node.get_logger().error("Invalid pick pose in result")
            # return False
            print("Debug ----------")
            pour_orientation = [0.707, 0.000, 0.707, 0.002]
            pour_pose.pose.orientation.x = pour_orientation[0]
            pour_pose.pose.orientation.y = pour_orientation[1]
            pour_pose.pose.orientation.z = pour_orientation[2]
            pour_pose.pose.orientation.w = pour_orientation[3]

        # 9. Pour Motion
        if not self.pour_motion(pour_pose, pick_result):
            self.node.get_logger().error("Pour motion failed")
            return False, pick_result.pick_result

        # 10. Return to Initial Pose Again
        time.sleep(5)
        self.node.get_logger().info("Returning to initial pose after pour")
        for i in range(5):
            res = send_joint_goal(
                move_joints_action_client=self.node._move_joints_client,
                named_position="table_stare",
                velocity=0.3,
            )
            if res:
                break

        self.node.get_logger().info("Pour Task Completed Successfully")
        return True, pick_result.pick_result


    def pick_motion(self, object_cluster: PointCloud2):
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

            # sort by score
            grasp_poses, grasp_scores = zip(
                *sorted(
                    zip(grasp_poses, grasp_scores),
                    key=lambda x: x[1],
                    reverse=True,
                )
            )
            grasp_poses, grasp_scores = grasp_poses[:10], grasp_scores[:10]
            
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
                    q_z_180_local = R.from_euler('z', 180, degrees=True)
                    q_result = (q_orig_rot * q_z_180_local).as_quat()  # [x, y, z, w]
                    reversed_pose.pose.orientation.x = q_result[0]
                    reversed_pose.pose.orientation.y = q_result[1]
                    reversed_pose.pose.orientation.z = q_result[2]
                    reversed_pose.pose.orientation.w = q_result[3]
                    new_grasp_poses.append(reversed_pose)
                    new_grasp_scores.append(grasp_score)
            else:
                new_grasp_poses = grasp_poses
                new_grasp_scores = grasp_scores

            if len(grasp_poses) == 0:
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
                time.sleep(2)

        if not pick_result_success:
            self.node.get_logger().error("Pick motion failed")
            return False, pick_result
        return True, pick_result

    def pour_motion(self, pose_msg: PoseStamped, pick_result) -> bool:
        self.node.get_logger().warning("FF 1.3")
        goal_msg = PourMotion.Goal(
            pour_pose=pose_msg,
            pick_result=pick_result.pick_result
        )

        self.node.get_logger().warning("FF 1.4")
        # Call Pour Motion Action
        self.node.get_logger().warning("Sending pour motion goal - FFN5")

        future = self.node._pour_motion_action_client.send_goal_async(goal_msg)
        self.node.get_logger().warning("after futurepour motion goal - FFN5")
        future = wait_for_future(future, timeout=120)

        pour_result = future.result().get_result().result
        self.node.get_logger().warning(f"Pour Motion Result: {pour_result}")
        
        self.node.get_logger().warning("Finished Pour Manager - FFN5.0")
        return True


    def wait_for_future(self, future):
        self.node.get_logger().warning("Waiting for future FF")
        if future is None:
            self.node.get_logger().info("Waiting for future FF1")
            self.node.get_logger().error("Service call failed: future is None")
            return False
        while not future:
            self.node.get_logger().info("Waiting for future FF2")
            pass
        # self.get_logger().info("Execution done with status: " + str(future.result()))
        return future  # 4 is the status for success

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

    def get_object_cluster(self, point: PointStamped, add_primitives: bool = True):
        request = PickPerceptionService.Request()
        request.point = point
        request.add_collision_objects = add_primitives
        self.node.pick_perception_3d_client.wait_for_service()
        future = self.node.pick_perception_3d_client.call_async(request)
        future = wait_for_future(future)

        pcl_result = future.result().cluster_result
        if len(pcl_result.data) == 0:
            self.node.get_logger().error("No object cluster detected")
            return None
        self.node.get_logger().info(
            f"Object cluster detected: {len(pcl_result.data)} points"
        )
        return pcl_result

    def calculate_object_height(self, obj_lowest, obj_highest):
        """Calculate the height of the object, measured from the lowest point to the highest point"""
        if obj_lowest.pose.header.frame_id != obj_highest.pose.header.frame_id:
            self.get_logger().error(
                "Object and pose frames do not match, cannot calculate height"
            )
            return 0.0
        obj_lowest_z = obj_lowest.pose.pose.position.z
        obj_highest_z = obj_highest.pose.pose.position.z
        obj_radius = obj_lowest.dimensions.x
        height = (obj_highest_z + obj_radius) - (obj_lowest_z - obj_radius)
        self.get_logger().info(f"Object height: {height}")
        return height

    def get_container_objects(self, container_name: str):
        """Obtiene los objetos de colisión del bowl."""
        request = GetCollisionObjects.Request()
        future = self.node._get_collision_objects_client.call_async(request)
        self.wait_for_future(future)

        if not future.result():
            self.node.get_logger().error("Error al obtener objetos de colisión")
            return None

        # Filtrar objetos del bowl por nombre
        return [
            obj for obj in future.result().collision_objects if container_name in obj.id
        ]
