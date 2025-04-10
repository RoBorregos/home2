from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import PerceptionService
from geometry_msgs.msg import PointStamped
from pick_and_place.utils.grasp_utils import get_grasps
from frida_interfaces.action import PickMotion

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
        if point is not None and (
            point.point.x != 0 and point.point.y != 0 and point.point.z != 0
        ):
            self.node.get_logger().info(f"Going for point: {point}")
        elif object_name is not None and object_name != "":
            self.node.get_logger().info(f"Going for object name: {object_name}")
            point = self.get_object_point(object_name)
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

        # Check result
        result = future.result()
        self.node.get_logger().info(f"Pick Motion Result: {result}")
        return result

    def get_object_point(self, object_name: str) -> PointStamped:
        return PointStamped()

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
