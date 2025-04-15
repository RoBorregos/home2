from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import PerceptionService, DetectionHandler
from geometry_msgs.msg import PointStamped
from pick_and_place.utils.grasp_utils import get_grasps
from frida_interfaces.action import PickMotion
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)

CFG_PATH = (
    "/workspace/src/home2/manipulation/packages/arm_pkg/config/frida_eigen_params.cfg"
)


class PickManager:
    node = None

    def __init__(self, node):
        print("Init pickmanager")
        self.node = node
        print("node:", self.node)

    def execute(self, object_name: str, point: PointStamped) -> bool:
        self.node.get_logger().info("Executing Pick Task")
        self.node.get_logger().info("Setting initial joint positions")
        # Set initial joint positions
        send_joint_goal(
            move_joints_action_client=self.node._move_joints_client,
            named_position="table_stare",
            velocity=0.3,
        )
        if point is not None and (
            point.point.x != 0 and point.point.y != 0 and point.point.z != 0
        ):
            self.node.get_logger().info(f"Going for point: {point}")
        elif object_name is not None and object_name != "":
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

        # Call Perception Service to get object cluster and generate collision objects
        object_cluster = self.get_object_cluster(point)
        if object_cluster is None:
            self.node.get_logger().error("No object cluster detected")
            return False

        # Call Grasp Pose Detection
        grasp_poses, grasp_scores = get_grasps(
            self.node.grasp_detection_client, object_cluster, CFG_PATH
        )

        if len(grasp_poses) == 0:
            self.node.get_logger().error("No grasp poses detected")
            return False

        # Call Pick Motion Action
        # Create goal
        goal_msg = PickMotion.Goal()
        goal_msg.grasping_poses = grasp_poses
        goal_msg.grasping_scores = grasp_scores

        # Send goal
        self.node.get_logger().info("Sending pick motion goal...")
        future = self.node._pick_motion_action_client.send_goal_async(goal_msg)
        future = wait_for_future(future)
        future = wait_for_future(future)
        # Check result
        result = future.result()
        self.node.get_logger().info(f"Pick Motion Result: {result}")
        self.node.get_logger().info("Returning to position")

        # return to configured position
        send_joint_goal(
            move_joints_action_client=self.node._move_joints_client,
            named_position="table_stare",
            velocity=0.3,
        )
        # Check result
        result = future.result().get_result().result

        return result.success

    def get_object_point(self, object_name: str) -> PointStamped:
        request = DetectionHandler.Request()
        request.label = object_name
        request.closest_object = False
        print("Request:", request)
        print("waiting for service")
        self.node.detection_handler_client.wait_for_service()
        future = self.node.detection_handler_client.call_async(request)
        print("waiting for future")
        future = wait_for_future(future)

        point = PointStamped()

        if len(future.result().detection_array.detections) == 1:
            self.node.get_logger().info(f"Object {object_name} found")
            point = future.result().detection_array.detections[0].point3d
        elif len(future.result().detection_array.detections) > 1:
            self.get_logger().info("Multiple objects found, selecting the closest one")
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
        request = PerceptionService.Request()
        request.point = point
        request.add_collision_objects = True
        self.node.perception_3d_client.wait_for_service()
        future = self.node.perception_3d_client.call_async(request)
        future = wait_for_future(future)

        pcl_result = future.result().cluster_result
        if len(pcl_result.data) == 0:
            self.node.get_logger().error("No object cluster detected")
            return None
        self.node.get_logger().info(
            f"Object cluster detected: {len(pcl_result.data)} points"
        )
        return pcl_result
