from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_motion_planning.utils.tf_utils import transform_pose, transform_point
from frida_interfaces.srv import (
    PlacePerceptionService,
    HeatmapPlace,
)
from geometry_msgs.msg import PoseStamped
from frida_interfaces.msg import PlaceParams
from frida_interfaces.action import PlaceMotion
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)
from frida_constants.manipulation_constants import TOP_SAFETY_MARGIN
from pick_and_place.utils.perception_utils import get_object_point, get_object_cluster
from frida_interfaces.msg import PickResult
from sensor_msgs_py import point_cloud2
import numpy as np
import json


def get_object_cloud_params(object_cluster):
    cloud_gen = point_cloud2.read_points(
        object_cluster, field_names=("x", "y", "z"), skip_nans=True
    )
    points = [list(p) for p in cloud_gen]
    if not points:
        return None

    points_np = np.array(points)
    centroid = np.mean(points_np, axis=0)
    max_z = np.max(points_np[:, 2])
    return centroid, max_z


def get_object_top_center(object_cluster, top_margin=0.03, node=None):
    """Estimate the center of the top opening of a convex object (e.g. trash can)."""
    cloud_gen = point_cloud2.read_points(
        object_cluster, field_names=("x", "y", "z"), skip_nans=True
    )
    points = [list(p) for p in cloud_gen]
    if not points:
        return None

    points_np = np.array(points)
    max_z = np.max(points_np[:, 2])

    # Radius from full cloud Y-extent (body has more coverage than rim)
    full_min_y = np.min(points_np[:, 1])
    full_max_y = np.max(points_np[:, 1])
    radius = (full_max_y - full_min_y) / 2.0
    center_y = (full_min_y + full_max_y) / 2.0

    # X: front edge of the object + radius = geometric center
    # Use 5th percentile instead of absolute min for noise robustness
    front_x = float(np.percentile(points_np[:, 0], 5))
    center_x = front_x + radius

    if node:
        node.get_logger().info(
            f"[top_center] radius={radius:.3f}, front_x={front_x:.3f}, "
            f"center=({center_x:.3f}, {center_y:.3f}), "
            f"cloud_size={len(points_np)}, top_points={np.sum(points_np[:, 2] >= max_z - top_margin)}"
        )

    center = np.array([center_x, center_y, max_z])
    return center, max_z


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
        
        start_position = "table_stare"
        return_position = "table_stare"
        if place_params.special_request != "":
            try:
                request_dict = json.loads(place_params.special_request)
                start_position = request_dict.get("start_position", start_position)
                return_position = request_dict.get("return_position", return_position)
            except Exception as e:
                self.node.get_logger().error(f"Failed to parse special_request: {e}")

        # Set initial joint positions
        if not place_params.is_shelf:
            send_joint_goal(
                move_joints_action_client=self.node._move_joints_client,
                named_position=start_position,
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

        return_result = True
        self.node.get_logger().info("Sending place motion request")
        self.node._place_motion_action_client.wait_for_server()
        future = self.node._place_motion_action_client.send_goal_async(
            place_motion_request
        )
        wait_for_future(future, timeout=20)
        try:
            return_result = future.result().get_result().result.success
        except Exception as e:
            return_result = False
            self.node.get_logger().error(f"Place motion failed: {e}")

        self.node.get_logger().info(f"Place Motion Result: {return_result}")

        self.node.get_logger().info("Returning to position")

        # return to configured position
        for i in range(5):
            back_res = send_joint_goal(
                move_joints_action_client=self.node._move_joints_client,
                named_position=return_position,
                velocity=0.5,
            )
            if back_res:
                self.node.get_logger().info("Returned to position successfully")
                break
            self.node.get_logger().info("Retry sending return joint goal")

        return return_result

    def get_place_pose(
        self, place_params: PlaceParams, pick_result: PickResult
    ) -> PoseStamped:
        result_pose = PoseStamped()
        place_on_top = False

        if place_params.forced_pose.header.frame_id != "":
            self.node.get_logger().info("Using forced place pose")
            result_pose = place_params.forced_pose
            # transform forced pose to the correct frame, mainly so we can alter orientation correctly
            # by default we go to the base_link frame
            # if we have pick_result, we use that frame -> most likely base_link too but we aim to be flexible

            if pick_result.pick_pose.header.frame_id != "":
                transform_frame = pick_result.pick_pose.header.frame_id
            else:
                transform_frame = "base_link"
            self.node.get_logger().info(
                f"Transforming forced place pose to {transform_frame} frame"
            )
            success, result_pose = transform_pose(
                result_pose, transform_frame, self.node.tf_buffer
            )
            if not success:
                self.node.get_logger().error("Failed to transform forced place pose")
                return None
            self.node.get_logger().info(
                f"Transformed forced place pose: {result_pose.pose.position.x}, "
                f"{result_pose.pose.position.y}, {result_pose.pose.position.z}"
            )
        else:
            heatmap_request = HeatmapPlace.Request()
            if place_params.close_to != "":
                close_by_object_name = place_params.close_to
                self.node.get_logger().info(
                    f"Using close to place pose for {place_params.close_to}"
                )

                close_by_object_name = place_params.close_to
                for i in range(5):
                    try:
                        self.node.get_logger().info("getting point")
                        close_by_point = get_object_point(
                            close_by_object_name,
                            self.node.detection_handler_client,
                        )
                        if close_by_point is None:
                            self.node.get_logger().warn(
                                f"Failed to get close by point for {close_by_object_name}, retrying..."
                            )
                        self.node.get_logger().info("getting cluster")
                        object_cluster = get_object_cluster(
                            close_by_point,
                            self.node.pick_perception_3d_client,
                            add_collision_objects=False,
                        )
                        if object_cluster is not None:
                            self.node.get_logger().info(
                                f"Object cluster detected for {close_by_object_name}"
                            )
                            self.node.remove_all_collision_object(attached=False)
                            # get centroid and use that as the close by point instead
                            object_centroid, max_z = get_object_cloud_params(
                                object_cluster
                            )
                            close_by_point.point.x = float(object_centroid[0])
                            close_by_point.point.y = float(object_centroid[1])
                            close_by_point.point.z = float(object_centroid[2])
                            close_by_point.header.frame_id = (
                                object_cluster.header.frame_id
                            )
                            self.node.get_logger().info(
                                f"Using centroid of object cluster as close by point: {close_by_point.point.x}, "
                                f"{close_by_point.point.y}, {close_by_point.point.z}"
                            )
                            break
                    except Exception as e:
                        self.node.get_logger().error(f"Failed to get object: {e}")

                transform_frame = "base_link"
                self.node.get_logger().info(
                    f"Transforming close by point to {transform_frame} frame"
                )
                success, close_by_point = transform_point(
                    close_by_point, transform_frame, self.node.tf_buffer
                )

                self.node.get_logger().info(f"Close by point: {close_by_point}")

                if not success:
                    self.node.get_logger().error("Failed to transform close by point")
                    return None

                heatmap_request.close_point = close_by_point
            # Call Perception Service to get object cluster and generate collision objects
            self.node.get_logger().info("Extracting table cloud")
            request = PlacePerceptionService.Request()
            request.place_params = place_params
            self.node.place_perception_3d_client.wait_for_service()
            future = self.node.place_perception_3d_client.call_async(request)
            wait_for_future(future, timeout=10)
            try:
                pcl_result = future.result().cluster_result
                if len(pcl_result.data) == 0:
                    self.node.get_logger().error("No plane cluster detected")
                    return None
            except Exception as e:
                self.node.get_logger().error(f"Failed to call perception service: {e}")
                return None
            pcl_result = future.result().cluster_result
            if len(pcl_result.data) == 0:
                self.node.get_logger().error("No plane cluster detected")
                return None

            self.node.get_logger().info(
                f"Plane cluster detected: {len(pcl_result.data)} points"
            )

            heatmap_request.pointcloud = pcl_result
            heatmap_request.prefer_closest = True
            if place_params.is_shelf:
                heatmap_request.prefer_closest = True
            self.node.place_pose_client.wait_for_service()
            future = self.node.place_pose_client.call_async(heatmap_request)
            wait_for_future(future)
            point_result = future.result().place_point

            heatmap_request = HeatmapPlace.Request()

            if place_params.special_request != "":
                request_dict = json.loads(place_params.special_request)
                request = request_dict.get("request", "")
                special_position = request_dict.get("position", "")
                max_distance = request_dict.get("max_distance", 0.3)
                max_height = request_dict.get("max_height", 0.2)
                close_by_point = None
                if request == "close_by":
                    self.node.get_logger().info(
                        f"Using close by place pose with max_distance={max_distance}, max_height={max_height}"
                    )
                    close_by_object_name = request_dict["object"]
                    for i in range(5):
                        try:
                            close_by_point = get_object_point(
                                close_by_object_name,
                                self.node.detection_handler_client,
                            )
                            if close_by_point is not None:
                                break
                            else:
                                self.node.get_logger().warn(
                                    f"Failed to get close by point for {close_by_object_name}, retrying..."
                                )
                        except Exception as e:
                            self.node.get_logger().error(
                                f"Failed to get object name: {e}"
                            )

                    transform_frame = "base_link"
                    self.node.get_logger().info(
                        f"Transforming close by point to {transform_frame} frame"
                    )
                    success, close_by_point = transform_point(
                        close_by_point, transform_frame, self.node.tf_buffer
                    )

                    self.node.get_logger().info(f"Close by point: {close_by_point}")

                    if not success:
                        self.node.get_logger().error(
                            "Failed to transform close by point"
                        )
                        return None

                    if special_position == "top":
                        result_pose.header.frame_id = close_by_point.header.frame_id
                        result_pose.pose.position.x = close_by_point.point.x
                        result_pose.pose.position.y = close_by_point.point.y
                        top_z = close_by_point.point.z
                        try:
                            if (
                                "object_cluster" not in locals()
                                or object_cluster is None
                            ):
                                object_cluster = get_object_cluster(
                                    close_by_point,
                                    self.node.pick_perception_3d_client,
                                    add_collision_objects=False,
                                )

                            if object_cluster is not None:
                                top_params = get_object_top_center(object_cluster, node=self.node)
                                if top_params is not None:
                                    center, max_z = top_params
                                    result_pose.pose.position.x = float(center[0])
                                    result_pose.pose.position.y = float(center[1])
                                    top_z = float(max_z)
                                    self.node.get_logger().info(
                                        f"Using estimated top center xy=({center[0]:.3f}, {center[1]:.3f}), top z={top_z:.3f}"
                                    )
                        except Exception as e:
                            self.node.get_logger().warn(
                                f"Could not compute cluster params, using detection point: {e}"
                            )

                        result_pose.pose.position.z = top_z + max_height + TOP_SAFETY_MARGIN
                        self.node.get_logger().info(
                            f"Top placement z={result_pose.pose.position.z:.3f} (top_z={top_z:.3f} + max_height={max_height} + margin={TOP_SAFETY_MARGIN})"
                        )
                        place_on_top = True
                    elif (
                        close_by_point is not None
                        and close_by_point.header.frame_id != ""
                    ):
                        heatmap_request.close_point = close_by_point
                        heatmap_request.special_request = place_params.special_request
                        self.node.get_logger().info(
                            f"Position '{special_position}' using max_distance={max_distance} for heatmap search"
                        )

            # if the task is not a forced pose or place on top of an object,
            # we use the perception service to get optimal place pose
            if result_pose.header.frame_id == "":
                self.node.get_logger().info("Extracting table cloud")

                request = PlacePerceptionService.Request()
                request.place_params = place_params
                self.node.place_perception_3d_client.wait_for_service()
                future = self.node.place_perception_3d_client.call_async(request)
                wait_for_future(future, timeout=10)
                try:
                    pcl_result = future.result().cluster_result
                    if len(pcl_result.data) == 0:
                        self.node.get_logger().error("No plane cluster detected")
                        return None
                except Exception as e:
                    self.node.get_logger().error(
                        f"Failed to call place perception service: {e}"
                    )
                    return None

                self.node.get_logger().info(
                    f"Plane cluster detected: {len(pcl_result.data)} points"
                )

                heatmap_request.pointcloud = pcl_result

                if place_params.is_shelf:
                    heatmap_request.prefer_closest = True
                self.node.place_pose_client.wait_for_service()
                future = self.node.place_pose_client.call_async(heatmap_request)
                wait_for_future(future)
                point_result = future.result().place_point

                self.node.get_logger().info(
                    f"Place point detected: {point_result.point.x}, {point_result.point.y}, {point_result.point.z}"
                )

                result_pose.header = point_result.header
                result_pose.pose.position = point_result.point

        if (
            pick_result.object_pick_height == 0
            or pick_result.pick_pose.header.frame_id == ""
        ):
            self.node.get_logger().warn("No object height detected using default")
            pick_result.object_pick_height = 0.15 if place_params.is_shelf else 0.10
            # z aiming down
            orientation_quat = [0.0, 1.0, 0.0, 0.0]
            pick_result.pick_pose.pose.orientation.x = orientation_quat[0]
            pick_result.pick_pose.pose.orientation.y = orientation_quat[1]
            pick_result.pick_pose.pose.orientation.z = orientation_quat[2]
            pick_result.pick_pose.pose.orientation.w = orientation_quat[3]
        else:
            self.node.get_logger().info(
                f"Object height detected: {pick_result.object_pick_height}"
            )

        if not place_on_top:
            result_pose.pose.position.z += (
                pick_result.object_pick_height if not place_params.is_shelf else 0.1
            )
        result_pose.pose.orientation = pick_result.pick_pose.pose.orientation

        return result_pose
