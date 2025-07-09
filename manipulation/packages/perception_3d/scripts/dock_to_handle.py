import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import Dock
from frida_interfaces.srv import DockToHandle

from rclpy.qos import QoSProfile
# import time


class DockToHandleNode(Node):
    def __init__(self):
        super().__init__("dock_to_handle_server")

        # Internal pose buffer
        self.latest_pose = None
        self.pose_received = False

        # Callback groups
        self.cb_group = ReentrantCallbackGroup()

        # Subscribe to the handle pose
        qos = QoSProfile(depth=10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/door_handle_pose",
            self.pose_callback,
            qos,
            callback_group=self.cb_group,
        )

        # Create Docking action client
        self.dock_client = ActionClient(
            self, Dock, "dock", callback_group=self.cb_group
        )

        # Create service
        self.srv = self.create_service(
            DockToHandle,
            "/dock_to_door_handle",
            self.handle_dock_request,
            callback_group=self.cb_group,
        )

        self.get_logger().info("🟢 DockToHandle service ready")

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        self.pose_received = True

    def handle_dock_request(self, request, response):
        self.get_logger().info("📨 Received docking request")

        # Wait for pose
        retries = 30
        # rate = self.create_rate(10)
        while not self.pose_received and retries > 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            retries -= 1

        if not self.pose_received:
            response.success = False
            response.message = "❌ No pose received on /door_handle_pose"
            self.get_logger().warn(response.message)
            return response

        # Wait for dock action server
        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            response.success = False
            response.message = "❌ Dock action server not available"
            self.get_logger().error(response.message)
            return response

        # Send dock goal
        goal_msg = Dock.Goal()
        goal_msg.docking_pose = self.latest_pose

        self.get_logger().info("🚀 Sending dock goal to Nav2")
        future_goal = self.dock_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future_goal)
        goal_handle = future_goal.result()

        if not goal_handle.accepted:
            response.success = False
            response.message = "❌ Docking goal rejected"
            self.get_logger().error(response.message)
            return response

        result_future = goal_handle.get_result_async()
        self.get_logger().info("⏳ Waiting for docking result...")
        rclpy.spin_until_future_complete(self, result_future)

        # result = result_future.result().result
        status = result_future.result().status

        if status == 4:  # SUCCEEDED
            response.success = True
            response.message = "✅ Successfully docked to handle"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"❌ Docking failed with status {status}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DockToHandleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
