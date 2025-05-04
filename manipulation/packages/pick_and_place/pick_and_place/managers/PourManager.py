from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import PickPerceptionService, DetectionHandler
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool
from pick_and_place.utils.grasp_utils import get_grasps
from frida_interfaces.action import PickMotion, PourMotion
from frida_interfaces.srv import GetCollisionObjects
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)


CFG_PATHS = [
    "/workspace/src/home2/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper_testing.cfg",
    "/workspace/src/home2/manipulation/packages/arm_pkg/config/frida_eigen_params_custom_gripper.cfg",
]


class PourManager:
    node = None

    def __init__(self, node):
        self.node = node

    def execute(self, object_name: str, container_object_name: str) -> bool:
        self.node.get_logger().info("Executing Pour Task")
        self.node.get_logger().info("Setting initial joint positions")
        point = self.get_object_point(object_name)
        if not point or point.point.x == 0.0:  # Check for invalid point
            self.node.get_logger().error(f"Invalid point for {object_name}")
            return False

        # Get container point
        container_point = self.get_object_point(container_object_name)
        if not container_point or container_point.point.x == 0.0:
            self.node.get_logger().error("Invalid container point")
            return False

        # Validate clusters
        object_cluster = self.get_object_cluster(point)
        container_cluster = self.get_object_cluster(container_point)
        if not object_cluster or not container_cluster:
            return False
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

        # get bowl top height
        bowl_objects = self.get_container_objects(container_object_name)
        if not bowl_objects:
            return False

        obj_lowest = min(bowl_objects, key=lambda o: o.pose.pose.position.z)
        obj_highest = max(bowl_objects, key=lambda o: o.pose.pose.position.z)

        # Call Perception Service to get object cluster and generate collision objects
        object_cluster = self.get_object_cluster(point)
        if object_cluster is None:
            self.node.get_logger().error("No object cluster detected")
            return False

        container_cluster = self.get_object_cluster(container_point)
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

        if not pick_result_success:
            self.node.get_logger().error("Pick motion failed")
            return False

        ### Up until here same as PickManager
        # Now go to pour
        # Send bowl object point to pour
        object_top_height = pick_result.object_height
        object_centroid_height = pick_result.object_pick_height
        bowl_top_height = self.calculate_object_height(obj_lowest, obj_highest)

        if bowl_top_height is None:
            return False

        bowl_centroid_height = container_point

        goal_msg = PourMotion.Goal(
            object_name=object_name,
            object_top_height=object_top_height,
            object_centroid_height=object_centroid_height,
            bowl_top_height=bowl_top_height,
            bowl_position=bowl_centroid_height,
        )

        self.node.get_logger().info("Sending pour motion goal...")

        future = self.node._pour_motion_action_client.send_goal_async(goal_msg)
        future = wait_for_future(future)

        if not future.result().result.success:
            self.node.get_logger().error("Pour motion failed")
            return False

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
        future = self.node.get_collision_objects_client.call_async(request)
        future = wait_for_future(future)

        if not future.result():
            self.node.get_logger().error("Error al obtener objetos de colisión")
            return None

        # Filtrar objetos del bowl por nombre
        return [
            obj for obj in future.result().collision_objects if container_name in obj.id
        ]
