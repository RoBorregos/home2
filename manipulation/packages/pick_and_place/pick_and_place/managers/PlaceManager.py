from frida_motion_planning.utils.ros_utils import wait_for_future
from frida_motion_planning.utils.tf_utils import transform_pose, transform_point
from frida_interfaces.srv import (
    PlacePerceptionService,
    HeatmapPlace,
    DetectionHandler,
    PickPerceptionService,
)
from frida_interfaces.msg import PlaceParams
from frida_interfaces.action import PlaceMotion
from frida_motion_planning.utils.service_utils import (
    move_joint_positions as send_joint_goal,
)
from pick_and_place.utils.perception_utils import get_object_point
from frida_interfaces.msg import PickResult
import json
from sensor_msgs_py import point_cloud2

from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import tf2_ros
import rclpy
from tf2_ros import TransformException, Buffer
from typing import Tuple
from tf2_geometry_msgs import do_transform_point


def transform_point(
    point: PointStamped, target_frame: str, tf_buffer: Buffer
) -> Tuple[bool, PointStamped]:
    """
    Transforms a point to a target frame using ROS2 tf2.
    Args:
        point (PointStamped): The point to transform.
        target_frame (str): The target frame to transform the point to.
        tf_buffer (Buffer): The tf2 buffer to use for looking up transforms.
    Returns:
        PointStamped: The transformed point.
    """
    success = False
    transformed_point = PointStamped()
    for i in range(5):
        try:
            # Wait for the transform to be available
            t = tf_buffer.lookup_transform(
                target_frame,
                point.header.frame_id,
                tf2_ros.Time(),
                timeout=rclpy.duration.Duration(seconds=3.0),
            )
            transformed_point = do_transform_point(point, t)
            transformed_point.header.frame_id = target_frame
            success = True
        except TransformException as e:
            print(
                f"Transform from {point.header.frame_id} to {target_frame} not available: {e}"
            )
        except Exception as e:
            print(f"An error occurred while transforming point: {e}")

    return success, transformed_point


def get_object_point(object_name: str, detection_handler_client) -> PointStamped:
    request = DetectionHandler.Request()
    request.label = object_name
    request.closest_object = False
    detection_handler_client.wait_for_service()
    future = detection_handler_client.call_async(request)
    future = wait_for_future(future, timeout=2.0)
    point = PointStamped()
    if not future:
        return point

    if len(future.result().detection_array.detections) == 1:
        point = future.result().detection_array.detections[0].point3d
    elif len(future.result().detection_array.detections) > 1:
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

    return point


def get_object_cluster(
    point: PointStamped, perception_3d_client, add_collision_objects=True
):
    request = PickPerceptionService.Request()
    request.point = point
    request.add_collision_objects = add_collision_objects
    perception_3d_client.wait_for_service()
    future = perception_3d_client.call_async(request)
    future = wait_for_future(future)

    pcl_result = future.result().cluster_result
    if len(pcl_result.data) == 0:
        return None
    return pcl_result


def get_object_centroid(object_cluster):
    # 8. Compute Centroid/Height and Set Pour Pose
    cloud_gen = point_cloud2.read_points(
        object_cluster, field_names=("x", "y", "z"), skip_nans=True
    )
    points = [list(p) for p in cloud_gen]
    if not points:
        return None

    points_np = np.array(points)
    centroid = np.mean(points_np, axis=0)
    return centroid


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
        if not place_params.is_shelf:
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
                named_position="table_stare",
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
                            object_centroid = get_object_centroid(object_cluster)
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
            heatmap_request = HeatmapPlace.Request()

            if place_params.special_request != "":
                request_dict = json.loads(place_params.special_request)
                request = request_dict.get("request", "")
                location = request_dict.get("location", "")
                close_by_point = None
                if request == "close_by":
                    self.node.get_logger().info("Using close by place pose")
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

                    if location == "top":
                        # we want to place on top of the object
                        result_pose.header.frame_id = close_by_point.header.frame_id
                        result_pose.pose.position.x = close_by_point.point.x
                        result_pose.pose.position.y = close_by_point.point.y
                        result_pose.pose.position.z = close_by_point.point.z
                    elif (
                        close_by_point is not None
                        and close_by_point.header.frame_id != ""
                    ):
                        heatmap_request.close_point = close_by_point
                        heatmap_request.special_request = place_params.special_request

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
            heatmap_request.prefer_closest = True
            if place_params.is_shelf:
                heatmap_request.prefer_closest = True
            self.node.place_pose_client.wait_for_service()
            future = self.node.place_pose_client.call_async(heatmap_request)
            wait_for_future(future)
            point_result = future.result().place_point
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

        # forget height if placing on shelf
        result_pose.pose.position.z += (
            pick_result.object_pick_height if not place_params.is_shelf else 0.1
        )
        result_pose.pose.orientation = pick_result.pick_pose.pose.orientation

        return result_pose
