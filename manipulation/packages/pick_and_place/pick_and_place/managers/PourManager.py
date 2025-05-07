from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import PickPerceptionService, DetectionHandler
from geometry_msgs.msg import PointStamped, PoseStamped
from std_srvs.srv import SetBool
from pick_and_place.utils.grasp_utils import get_grasps
from frida_interfaces.action import PickMotion, MoveToPose
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)
from frida_constants.manipulation_constants import (
    PICK_VELOCITY,
    PICK_ACCELERATION,
    PICK_PLANNER,
    GRASP_LINK_FRAME,
)
from sensor_msgs_py import point_cloud2
import time

CFG_PATHS = [
    "/workspace/src/home2/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper_testing.cfg",
    "/workspace/src/home2/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper.cfg",
]


class PourManager:
    node = None

    def __init__(self, node):
        self.node = node

    def execute(self, pour_params) -> bool:
        self.node.get_logger().info("Executing Pour Task")
        self.node.get_logger().info("Setting initial joint positions")
        object_name = pour_params.object_name
        container_object_name = pour_params.container_name
        # time.sleep(10)
        # Set initial joint positions
        send_joint_goal(
            move_joints_action_client=self.node._move_joints_client,
            named_position="table_stare",
            velocity=0.3,
        )
        if (object_name is not None and object_name != "") or (
            container_object_name is not None and container_object_name != ""
        ):
            self.node.get_logger().info(f"Going for object name: {object_name}")
            point = self.get_object_point(object_name)
            if point is None:
                self.node.get_logger().error(
                    f"Object {object_name} not found, please provide a point"
                )
                return False
        else:
            self.node.get_logger().error("No object name or point provided")
            return False

        # get bowl object point
        if container_object_name is not None and container_object_name != "":
            self.node.get_logger().info(
                f"Going for bowl object name: {container_object_name}"
            )
            container_point = self.get_object_point(container_object_name)
            if container_point is None:
                self.node.get_logger().error(
                    f"Bowl object {container_object_name} not found, please provide a point"
                )
                return False
        else:
            self.node.get_logger().error("No bowl object name or point provided")
            return False

        # Call Perception Service to get object cluster and generate collision objects
        object_cluster = self.get_object_cluster(point, add_primitives=True)
        if object_cluster is None:
            self.node.get_logger().error("No object cluster detected")
            return False

        container_cluster = self.get_object_cluster(
            container_point, add_primitives=False
        )
        if container_cluster is None:
            self.node.get_logger().error("No container cluster detected")
            return False

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
            self.node.get_logger().info(f"CFG_PATH: {CFG_PATH}")
            # Call Grasp Pose Detection
            grasp_poses, grasp_scores = get_grasps(
                self.node.grasp_detection_client, object_cluster, CFG_PATH
            )

            grasp_poses, grasp_scores = grasp_poses[:5], grasp_scores[:5]

            if len(grasp_poses) == 0:
                self.node.get_logger().error("No grasp poses detected")
                continue

            # Call Pick Motion Action

            # Create goal
            goal_msg = PickMotion.Goal()
            goal_msg.grasping_poses = grasp_poses
            goal_msg.grasping_scores = grasp_scores

            # Send goal
            self.node.get_logger().info("Sending pick motion goal...")
            future = self.node._pick_motion_action_client.send_goal_async(goal_msg)
            future = wait_for_future(future)
            # Check result
            pick_result = future.result().get_result().result
            self.node.get_logger().info(f"Pick Motion Result: {pick_result}")
            if pick_result.success:
                pick_result_success = True
                break
            time.sleep(1)

        if not pick_result_success:
            self.node.get_logger().error("Pick motion failed")
            return False

        ### Up until here same as PickManager
        # Now go to pour
        self.pour(container_cluster, pick_result)

        self.node.get_logger().info("Returning to position")

        for i in range(5):
            # return to configured position
            return_result = send_joint_goal(
                move_joints_action_client=self.node._move_joints_client,
                named_position="table_stare",
                velocity=0.3,
            )
            if return_result:
                break
        # self.node.get_logger().info("Waiting for 10 seconds")
        # time.sleep(10)
        return result.success

    def pour(self, container_cluster, pick_result):
        point_cloud = container_cluster
        if point_cloud.header.frame_id != "base_link":
            self.get_logger().warn(
                f"PointCloud2 frame_id is {point_cloud.header.frame_id}, expected base_link"
            )
            return False, None
        cloud_gen = point_cloud2.read_points(
            point_cloud, field_names=("x", "y", "z"), skip_nans=True
        )
        # find middle point and top height
        x_sum = 0
        y_sum = 0
        z_sum = 0
        top_z = 0
        count = 0
        for p in cloud_gen:
            x_sum += p[0]
            y_sum += p[1]
            z_sum += p[2]
            if p[2] > top_z:
                top_z = p[2]
            count += 1
        if count == 0:
            self.node.get_logger().error("No points in point cloud")
            return False, None
        x_avg = x_sum / count
        y_avg = y_sum / count
        z_avg = z_sum / count
        self.node.get_logger().info(f"Average point: {x_avg}, {y_avg}, {z_avg}")
        self.node.get_logger().info(f"Top point: {top_z}")
        # create point
        pour_point = PointStamped()
        pour_point.header.frame_id = "base_link"
        pour_point.header.stamp = self.node.get_clock().now().to_msg()
        pour_point.point.x = x_avg
        pour_point.point.y = y_avg
        pour_point.point.z = top_z + 0.1
        self.node.get_logger().info(f"Pour point: {pour_point}")
        # Call pour motion -> this will go somewhere else eventually idc right now
        pour_pose = PoseStamped()
        pour_pose.header.frame_id = "base_link"
        pour_pose.header.stamp = self.node.get_clock().now().to_msg()
        pour_pose.pose.position.x = pour_point.point.x
        pour_pose.pose.position.y = pour_point.point.y
        pour_pose.pose.position.z = pour_point.point.z

        pour_pose.pose.orientation = pick_result.pick_pose.pose.orientation

        tries = 3
        distance_between_tries = 0.1
        for i in range(tries):
            pour_pose.pose.position.z += distance_between_tries
            self.node.get_logger().info(f"Pour pose: {pour_pose}")
            # Call pour motion action
            pour_pose_handler, pour_pose_result = self.move_to_pose(pour_pose)

            if pour_pose_result.result.success:
                self.node.get_logger().info("Pour motion succeeded")
                break
            else:
                self.node.get_logger().error("Pour motion failed")
                if i == tries - 1:
                    self.node.get_logger().error("Pour motion failed after 3 tries")
                    return False, None
                else:
                    self.node.get_logger().info(
                        f"Pour motion failed, trying again with z = {pour_pose.pose.position.z}"
                    )
        time.sleep(10.0)

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

    def move_to_pose(self, pose):
        """Move the robot to the given pose."""
        request = MoveToPose.Goal()
        request.pose = pose
        request.velocity = PICK_VELOCITY
        request.acceleration = PICK_ACCELERATION
        request.planner_id = PICK_PLANNER
        request.target_link = GRASP_LINK_FRAME
        future = self._move_to_pose_action_client.send_goal_async(request)
        self.wait_for_future(future)
        action_result = future.result().get_result()
        return future.result(), action_result
