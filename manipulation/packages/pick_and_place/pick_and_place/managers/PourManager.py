import numpy as np
import rclpy
from frida_pymoveit2.robots import xarm6 as robot
from rclpy.duration import Duration
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
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
from frida_interfaces.action import PickMotion, PourMotion, MoveToPose
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
from sensor_msgs.msg import PointCloud2
import time

CFG_PATHS = [
    "/workspace/src/home2/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper_testing.cfg",
    "/workspace/src/home2/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper.cfg",
]


class PourManager:
    node = None

    def __init__(self, node):
        self.node = node

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

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
            return False

        # 3. Get Bowl Point
        bowl_point = self.get_object_point(container_object_name)
        if bowl_point is None:
            self.node.get_logger().error(f"Bowl {container_object_name} not found")
            return False

        # 4. Get Clusters
        object_cluster = self.get_object_cluster(object_point, add_primitives=True)
        if object_cluster is None:
            return False

        bowl_cluster = self.get_object_cluster(bowl_point, add_primitives=False)
        if bowl_cluster is None:
            return False

        # 5. Open Gripper
        gripper_request = SetBool.Request()
        gripper_request.data = True
        future = self.node._gripper_set_state_client.call_async(gripper_request)
        wait_for_future(future)

        # 6. Pick Motion
        pick_success, pick_result = self.pick_motion(object_cluster)
        if not pick_success:
            return False

        # 7. Return to Initial Pose
        send_joint_goal(
            move_joints_action_client=self.node._move_joints_client,
            named_position="table_stare",
            velocity=0.3,
        )

        # 8. Compute Centroid/Height and Set Pour Pose
        cloud_gen = point_cloud2.read_points(
            bowl_cluster, field_names=("x", "y", "z"), skip_nans=True
        )
        points = [list(p) for p in cloud_gen]
        if not points:
            self.node.get_logger().error("Empty bowl cluster")
            return False

        points_np = np.array(points)
        centroid = np.mean(points_np, axis=0)
        max_z = np.max(points_np[:, 2])
        self.node.get_logger().info(f"Bowl Centroid: {centroid}, Max Z: {max_z}")

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=robot.base_link_name(),
                source_frame=bowl_point.header.frame_id,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            transformed_point = do_transform_point(bowl_point, transform)
        except Exception as e:
            self.node.get_logger().error(f"Transform error: {repr(e)}")
            return False

        pour_pose = PoseStamped()
        pour_pose.header.frame_id = robot.base_link_name()
        pour_pose.header.stamp = self.node.get_clock().now().to_msg()
        pour_pose.pose.position = transformed_point.point
        pour_pose.pose.position.z += 0.05  # Slightly above the bowl

        pick_pose = getattr(pick_result.pick_result, "pick_pose", None)
        if pick_pose:
            pour_pose.pose.orientation = pick_pose.pose.orientation
        else:
            self.node.get_logger().error("Invalid pick pose in result")
            return False

        # 9. Pour Motion
        if not self.pour_motion(pour_pose):
            return False

        # 10. Return to Initial Pose Again
        send_joint_goal(
            move_joints_action_client=self.node._move_joints_client,
            named_position="table_stare",
            velocity=0.3,
        )

        self.node.get_logger().info("Pour Task Completed Successfully")
        return True


    def pick_motion(self, object_cluster: PointCloud2):
        # open gripper
        self.node.get_logger().warning("Pick Action - FFN6")
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
            return False, pick_result
        return True, pick_result

    def pour_motion(self, pose_msg: PoseStamped):
        self.node.get_logger().warning("FF 1.3")
        goal_msg = PourMotion.Goal(
            pour_pose=pose_msg,
        )

        self.node.get_logger().warning("FF 1.4")
        # Call Pour Motion Action
        self.node.get_logger().warning("Sending pour motion goal - FFN5")

        future = self.node._pour_motion_action_client.send_goal_async(goal_msg)
        self.node.get_logger().warning("after futurepour motion goal - FFN5")
        future = self.wait_for_future(future)

        # Print the result of the pour motion?
        self.node.get_logger().warning("Result of pour motion - FFN5")

        # result_pour = future.result().get_result()
        result_pour = future.result()
        if not result_pour.result.success:
            self.node.get_logger().error("Pour motion failed")
            return False
        
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
