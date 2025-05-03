#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.srv import GetPlaneBbox, GetOptimalPositionForPlane
from frida_interfaces.action import MoveToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PointStamped
from frida_motion_planning.utils.tf_utils import look_at
from frida_pymoveit2.robots import xarm6
import time

TABLE_Z_OFFSET = 0.5
SHELF_Z_OFFSET = 0.3


class MyPoint:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def from_point(self, point):
        self.x = point.x
        self.y = point.y
        self.z = point.z
        return self

    def __repr__(self):
        return f"Point(x={self.x}, y={self.y}, z={self.z})"

    def __eq__(self, other):
        if not isinstance(other, MyPoint):
            return False
        return self.x == other.x and self.y == other.y and self.z == other.z


class FixPositionToPlane(Node):
    def __init__(self):
        super().__init__("fix_position_to_plane")
        self.call_bck_group = ReentrantCallbackGroup()
        self.get_plane_bbox_client = self.create_client(
            GetPlaneBbox, "/manipulation/get_plane_bbox"
        )
        self.opt_plane_pose_srv = self.create_service(
            GetOptimalPositionForPlane,
            "/manipulation/get_optimal_position_for_plane",
            self.get_optimal_position_for_plane_callback,
            callback_group=self.call_bck_group,
        )
        self._action_client = ActionClient(
            self,
            MoveToPose,
            "/manipulation/move_to_pose_action_server",
            callback_group=self.call_bck_group,
        )
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        while not self.get_plane_bbox_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "get_plane_bbox service not available, waiting again..."
            )
        self.get_logger().info("get_plane_bbox service is available")

    def callback_call_service(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response}")
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")

    def broadcast_transform(self, point, quaternion, parent_frame, child_frame):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        # Set translation
        transform.transform.translation.x = point.point.x
        transform.transform.translation.y = point.point.y
        transform.transform.translation.z = point.point.z

        # Set rotation
        transform.transform.rotation.x = quaternion.quaternion.x
        transform.transform.rotation.y = quaternion.quaternion.y
        transform.transform.rotation.z = quaternion.quaternion.z
        transform.transform.rotation.w = quaternion.quaternion.w

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def get_line_from_points(p1: MyPoint, p2: MyPoint, distance: float):
        # y = mx + b
        m = (p2.y - p1.y) / (p2.x - p1.x)
        b = p1.y - m * p1.x
        print(f"m: {m}, b: {b}")
        k = p1.y - b

        # d = (p2.x*100 - p1.x*100) ** 2 + (p2.y*100 - p1.y*100) ** 2
        # print(f"d: {d}")
        # distance *= 100
        # print(f"distance: {distance}")
        # distance += d
        # print(f"distance: {distance}")
        # distance = distance / 100
        # print(f"distance: {distance}")

        d = ((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2) ** 0.5
        print(f"d: {d}")
        distance += d
        print(f"distance: {distance}")
        A = m**2 + 1
        B = -2 * (p1.x + k * m)
        C = p1.x**2 + k**2 - distance**2
        print(f"A: {A}, B: {B}, C: {C}")
        # x = (-B + (B**2 - 4*A*C)**0.5) / (2*A)
        if B**2 - 4 * A * C < 0:
            print("No solution")
            return None
        x = (-B - (B**2 - 4 * A * C) ** 0.5) / (2 * A)
        y = m * x + b
        z = p1.z
        new_point = MyPoint(x, y, z)
        print(f"New point: {new_point}")
        return new_point

    def wait_for_future(self, future, timeout=5):
        start_time = time.time()
        while future is None and (time.time() - start_time) < timeout:
            pass
        if future is None:
            return False
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.1)
            # print("Waiting for future to complete...")
        return future

    def get_optimal_position_for_plane_callback(
        self,
        request: GetOptimalPositionForPlane.Request,
        response: GetOptimalPositionForPlane.Response,
    ) -> GetOptimalPositionForPlane.Response:
        self.get_logger().info("Received request for optimal position")
        max_h, min_h = request.plane_est_max_height, request.plane_est_min_height
        self.get_logger().info(f"Max height: {max_h}, Min height: {min_h}")

        # Call the get_plane_bbox service
        self.get_logger().info("Calling get_plane_bbox service")
        plane_bbox_request = GetPlaneBbox.Request()
        plane_bbox_request.max_height = max_h
        plane_bbox_request.min_height = min_h
        ivan = TABLE_Z_OFFSET if request.table_or_shelf else SHELF_Z_OFFSET

        future = self.get_plane_bbox_client.call_async(plane_bbox_request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        self.wait_for_future(future)
        res = future.result()
        if res is None or res.health_response != 0:
            # self.get_logger().info("Received response from get_plane_bbox service")
            self.get_logger().error("Error while calling get_plane_bbox service")
            response.is_valid = False
            return response

        self.get_logger().info("Received response from get_plane_bbox service")
        self.get_logger().info(
            f"Plane bbox2: {res.center}, HRES: {res.health_response}"
        )
        try:
            center = MyPoint().from_point(res.center.point)
            p1 = MyPoint().from_point(res.pt1.point)
            p2 = MyPoint().from_point(res.pt2.point)
            p3 = MyPoint().from_point(res.pt3.point)
            p4 = MyPoint().from_point(res.pt4.point)

            self.get_logger().info(f"p1: {p1}, p2: {p2}, p3: {p3}, p4: {p4}")

            p12 = MyPoint((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, (p1.z + p2.z) / 2)
            p34 = MyPoint((p3.x + p4.x) / 2, (p3.y + p4.y) / 2, (p3.z + p4.z) / 2)
            p23 = MyPoint((p2.x + p3.x) / 2, (p2.y + p3.y) / 2, (p2.z + p3.z) / 2)
            p41 = MyPoint((p4.x + p1.x) / 2, (p4.y + p1.y) / 2, (p4.z + p1.z) / 2)

            self.get_logger().info(f"p12: {p12}, p34: {p34}, p23: {p23}, p41: {p41}")

            p_12 = MyPoint().from_point(self.get_line_from_points(center, p12, 0.35))
            p_34 = MyPoint().from_point(self.get_line_from_points(center, p34, 0.35))
            p_23 = MyPoint().from_point(self.get_line_from_points(center, p23, 0.35))
            p_41 = MyPoint().from_point(self.get_line_from_points(center, p41, 0.35))

            self.get_logger().info(
                f"p_12: {p_12}, p_34: {p_34}, p_23: {p_23}, p_41: {p_41}"
            )

            # get the closest point to (0, 0, 0)
            points = [p_12, p_34, p_23, p_41]
            closest_point = min(points, key=lambda p: (p.x**2 + p.y**2 + p.z**2) ** 0.5)
            self.get_logger().info(f"Closest point: {closest_point}")

            response.pt1.point.x = closest_point.x
            response.pt1.point.y = closest_point.y
            response.pt1.point.z = closest_point.z + ivan
            response.pt1.header.frame_id = "base_link"
            response.pt1.header.stamp = self.get_clock().now().to_msg()

            response.q1.header.frame_id = "base_link"
            response.q1.header.stamp = self.get_clock().now().to_msg()

            # # Set the quaternion to identity
            response.q1.quaternion.x = 0.0
            response.q1.quaternion.y = 0.0
            response.q1.quaternion.z = 0.0
            response.q1.quaternion.w = 1.0

            # jaw = (center.x - closest_point.x) / (
            #     (center.x - closest_point.x) ** 2
            #     + (center.y - closest_point.y) ** 2
            # ) ** 0.5

            # roll = 0
            # pitch = 0

            # if jaw > 0:
            #     response.q1.quaternion.x = 0.0
            #     response.q1.quaternion.y = 0.0
            #     response.q1.quaternion.z = jaw
            #     response.q1.quaternion.w = (1 - jaw**2) ** 0.5
            # else:
            #     response.q1.quaternion.x = 0.0
            #     response.q1.quaternion.y = 0.0
            #     response.q1.quaternion.z = -jaw
            #     response.q1.quaternion.w = (1 - jaw**2) ** 0.5

            response.is_valid = True

            # Broadcast the transform
            self.broadcast_transform(
                response.pt1, response.q1, "base_link", "optimal_position"
            )

            self.get_logger().info(
                f"Response: {response.pt1}, {response.q1}, {response.is_valid}"
            )

            center_point = PointStamped()
            center_point.header.frame_id = "base_link"
            center_point.header.stamp = self.get_clock().now().to_msg()
            center_point.point.x = center.x
            center_point.point.y = center.y
            center_point.point.z = center.z

            t = look_at(
                source_pose=response.pt1,
                target_pose=center_point,
            )

            req_move = MoveToPose.Goal()
            req_move.pose.header.frame_id = "base_link"
            req_move.pose.header.stamp = self.get_clock().now().to_msg()
            req_move.pose = t

            req_move.velocity = 0.3
            req_move.target_link = xarm6.camera_frame_name()

            # self.get_logger().info(f"Request: {req_move}")
            # self.get_logger().info(f"Request: {req_move}")
            fut = self._action_client.send_goal_async(req_move)
            fut.add_done_callback(self.callback_call_service)
            self.get_logger().info("Requested")
            return response
        except Exception as e:
            self.get_logger().error(f"Error while processing the request: {e}")
            response.is_valid = False
            return response
        self.get_logger().info("Response:a aaa")
        return response


def main(args=None):
    rclpy.init(args=args)
    fix_position_to_plane = FixPositionToPlane()
    executor = MultiThreadedExecutor(2)
    executor.add_node(fix_position_to_plane)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        fix_position_to_plane.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
