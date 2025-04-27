from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_interfaces.srv import PlacePerceptionService, HeatmapPlace
from geometry_msgs.msg import PoseStamped
from frida_interfaces.msg import PlaceParams
from frida_interfaces.action import PlaceMotion
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)
from frida_interfaces.msg import PickResult


class PlaceManager:
    node = None

    def __init__(self, node):
        self.node = node
        self.node.ee_link_posepub = self.node.create_publisher(
            PoseStamped,
            "/manipulator/place_ee_link_pose",
            10,
        )

    def execute(self, place_params: PlaceParams, pick_result: PickResult) -> bool:
        self.node.get_logger().info("Executing Place Task")
        self.node.get_logger().info("Setting initial joint positions")
        # Set initial joint positions
        send_joint_goal(
            move_joints_action_client=self.node._move_joints_client,
            named_position="table_stare",
            velocity=0.3,
        )

        # Call Perception Service to get object cluster and generate collision objects
        self.node.get_logger().info("Generating place pose")
        place_pose = self.get_place_pose(place_params, pick_result)

        self.node.get_logger().info(f"Place pose: {place_pose}")

        self.node.ee_link_posepub.publish(place_pose)

        place_motion_request = PlaceMotion.Goal()
        place_motion_request.place_pose = place_pose
        place_motion_request.object_name = pick_result.object_name
        place_motion_request.place_params = place_params

        self.node.get_logger().info("Sending place motion request")
        self.node._place_motion_action_client.wait_for_server()
        future = self.node._place_motion_action_client.send_goal_async(
            place_motion_request
        )
        wait_for_future(future)
        place_motion_result = future.result().get_result().result
        self.node.get_logger().info(f"Pick Motion Result: {place_motion_result}")

        if not place_motion_result.success:
            self.node.get_logger().error("Pick motion failed")
            return False

        self.node.get_logger().info("Returning to position")

        # return to configured position
        return_result = send_joint_goal(
            move_joints_action_client=self.node._move_joints_client,
            named_position="table_stare",
            velocity=0.3,
        )

        return return_result

    def get_place_pose(
        self, place_params: PlaceParams, pick_result: PickResult
    ) -> PoseStamped:
        # Call Perception Service to get object cluster and generate collision objects
        self.node.get_logger().info("Extracting table cloud")
        request = PlacePerceptionService.Request()
        request.place_params = place_params
        self.node.place_perception_3d_client.wait_for_service()
        future = self.node.place_perception_3d_client.call_async(request)
        wait_for_future(future)
        pcl_result = future.result().cluster_result
        if len(pcl_result.data) == 0:
            self.node.get_logger().error("No plane cluster detected")
            return None
        self.node.get_logger().info(
            f"Plane cluster detected: {len(pcl_result.data)} points"
        )

        heatmap_request = HeatmapPlace.Request()
        heatmap_request.pointcloud = pcl_result
        if place_params.is_shelf:
            heatmap_request.prefer_closest = True

        self.node.place_pose_client.wait_for_service()
        future = self.node.place_pose_client.call_async(heatmap_request)
        wait_for_future(future)
        point_result = future.result().place_point

        result_pose = PoseStamped()
        result_pose.header = point_result.header
        result_pose.pose.position = point_result.point

        if (
            pick_result.object_pick_height == 0
            or pick_result.pick_pose.header.frame_id == ""
        ):
            self.node.get_logger().warn("No object height detected using default")
            pick_result.object_pick_height = 0.15 if place_params.is_shelf else 0.20
            # z aiming down
            orientation_quat = [0.0, 1.0, 0.0, 0.0]
            pick_result.pick_pose.pose.orientation.x = orientation_quat[0]
            pick_result.pick_pose.pose.orientation.y = orientation_quat[1]
            pick_result.pick_pose.pose.orientation.z = orientation_quat[2]
            pick_result.pick_pose.pose.orientation.w = orientation_quat[3]

        # forget height if placing on shelf
        result_pose.pose.position.z += (
            pick_result.object_pick_height if not place_params.is_shelf else 0.15
        )
        result_pose.pose.orientation = pick_result.pick_pose.pose.orientation

        return result_pose
