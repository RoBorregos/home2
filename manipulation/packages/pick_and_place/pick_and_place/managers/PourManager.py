import numpy as np
from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import PickPerceptionService, DetectionHandler
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs_py import point_cloud2

# from std_srvs.srv import SetBool
# from pick_and_place.utils.grasp_utils import get_grasps
from frida_interfaces.action import PourMotion  # PickMotion
from frida_interfaces.srv import GetCollisionObjects
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)
from frida_pymoveit2.robots import xarm6 as robot

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

        # time.sleep(10)
        # Set initial joint positions
        self.node.get_logger().warning("Set initial joint positions - FFN1")
        send_joint_goal(
            move_joints_action_client=self.node._move_joints_client,
            named_position="table_stare",
            velocity=0.3,
        )

        # get bowl object point
        self.node.get_logger().warning("Get bowl object point - FFN2")
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

        # get cluster for bowl object
        self.node.get_logger().warning("Get cluster for bowl object - FFN3")
        container_cluster = self.get_object_cluster(container_point)
        if container_cluster is None:
            self.node.get_logger().error("No bowl object cluster detected")
            return False

        # TODO: computer the bowl top height since container cluster
        # TODO: computer the bowl centroid height since container cluster

        if container_cluster.header.frame_id != robot.base_link_name():
            self.get_logger().warn(
                f"PointCloud2 frame_id is {container_cluster.header.frame_id}, expected {robot.base_link_name()}"
            )
            return False
        else:
            self.get_logger().info(
                "PointCloud2 frame_id is {container_cluster.header.frame_id}"
            )

        cloud_gen = point_cloud2.read_points(
            container_cluster, field_names=("x", "y", "z"), skip_nans=True
        )

        points = []
        max_z = float("-inf")
        for p in cloud_gen:
            points.append([p[0], p[1], p[2]])
            if p[2] > max_z:
                max_z = p[2]

        if not points:
            self.get_logger().error("Empty cluster received")
            return False

        points_np = np.array(points)
        centroid = np.mean(points_np, axis=0)

        self.get_logger().info(f"Centroid: {centroid.point}")
        self.get_logger().info(f"Max Z: {max_z:.3f}")

        # Set the final pose
        self.node.get_logger().error("Setting final pose - FFN4")
        pose_msg = PoseStamped()
        pose_msg.msg.header.frame_id = robot.base_link_name()
        pose_msg.msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.msg.pose.position.x = centroid.point.x
        pose_msg.msg.pose.position.y = centroid.point.y + 0.05  # TODO: set the offset
        pose_msg.msg.pose.position.z = (
            float(max_z) + 0.15
        )  # TODO: set the height of the bowl
        pose_msg.msg.pose.orientation.x = 0.7071
        pose_msg.msg.pose.orientation.y = 0.0
        pose_msg.msg.pose.orientation.z = 0.0
        pose_msg.msg.pose.orientation.w = -0.7071
        self.node.get_logger().info(f"Final pose: {pose_msg}")

        goal_msg = PourMotion.Goal(
            pour_pose=pose_msg,
        )

        # Call Pour Motion Action
        self.node.get_logger().info("Sending pour motion goal - FFN5")
        future = self.node._pour_motion_action_client.send_goal_async(goal_msg)
        future = wait_for_future(future)

        # Print the result of the pour motion
        self.node.get_logger().error("Result of pour motion - FFN5")
        if not future.result().result.success:
            self.node.get_logger().error("Pour motion failed")
            return False

        # Return to the initial position
        self.node.get_logger().info("Returning to position - FFN6")
        for i in range(5):
            # return to configured position
            return_result = send_joint_goal(
                move_joints_action_client=self.node._move_joints_client,
                named_position="table_stare",
                velocity=0.3,
            )
            if return_result:
                self.node.get_logger().error("Se metio al fucking fucker - Error")
                break
        # self.node.get_logger().info("Waiting for 10 seconds")
        # time.sleep(10)
        self.node.get_logger().error("fucking fucker 6")
        return True

    def wait_for_future(self, future):
        if future is None:
            self.get_logger().error("Service call failed: future is None")
            return False
        while not future.done():
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

    def get_object_cluster(self, point: PointStamped):
        request = PickPerceptionService.Request()
        request.point = point
        request.add_collision_objects = False
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
