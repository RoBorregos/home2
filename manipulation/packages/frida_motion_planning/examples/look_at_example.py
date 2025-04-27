#!/usr/bin/env python3
# simple point follower listening to /odom and /published_point

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from frida_interfaces.action import MoveToPose
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from frida_motion_planning.utils.tf_utils import look_at
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from frida_pymoveit2.robots import xarm6


class LookAt(Node):
    def __init__(self):
        super().__init__("look_at_example")
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self,
            MoveToPose,
            "/manipulation/move_to_pose_action_server",
            callback_group=self.callback_group,
        )
        # Publisher for the debug PoseStamped topic
        self._debug_pub = self.create_publisher(
            PoseStamped, "/call_pose_goal/debug_pose", 10
        )
        self._clicked_point_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.point_callback,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_client.wait_for_server()
        self.get_logger().info("Service available, ready to receive goals")

    def point_callback(self, msg):
        target_point = msg

        self.get_logger().info("Requesting to follow point")

        # get current transform of camera
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",  # target frame
                xarm6.camera_frame_name(),  # source frame
                target_point.header.stamp,  # time
                rclpy.duration.Duration(seconds=0.5),  # timeout
            )
            self.get_logger().info(f"Camera Transform: {transform}")

        except TransformException as e:
            self.get_logger().error(f"Could not transform: {e}")
            return

        camera_pose = PoseStamped()
        camera_pose.header.frame_id = "base_link"
        camera_pose.header.stamp = target_point.header.stamp
        camera_pose.pose.position.x = transform.transform.translation.x
        camera_pose.pose.position.y = transform.transform.translation.y
        camera_pose.pose.position.z = transform.transform.translation.z

        target_pose = PoseStamped()
        target_pose.header.frame_id = target_point.header.frame_id
        target_pose.header.stamp = target_point.header.stamp
        target_pose.pose.position.x = target_point.point.x
        target_pose.pose.position.y = target_point.point.y
        target_pose.pose.position.z = target_point.point.z

        # Call the look_at function to get the new pose
        try:
            new_pose = look_at(camera_pose, target_pose)
            self.get_logger().info(f"New Pose: {new_pose}")
            target_pose = new_pose
        except Exception as e:
            self.get_logger().error(f"Error in look_at: {e}")
            return

        # Publish the debug pose
        self._debug_pub.publish(target_pose)

        request = MoveToPose.Goal()
        request.pose = target_pose
        request.target_link = xarm6.camera_frame_name()
        self.get_logger().info(f"Goal: {request}")

        future = self._action_client.send_goal_async(request)
        future.add_done_callback(self.callback_call_service)

    def callback_call_service(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response}")
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    look_at_node = LookAt()
    executor.add_node(look_at_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        look_at_node.destroy_node()
        rclpy.shutdown()
        look_at_node.get_logger().info("Node destroyed")
        look_at_node.get_logger().info("Node shutdown")


if __name__ == "__main__":
    main()
