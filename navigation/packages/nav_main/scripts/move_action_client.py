import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav_main.action import Move


class MoveActionClient(Node):

    def __init__(self):
        super().__init__('Move_action_client')
        self._action_client = ActionClient(self, Move, 'move')

    def send_goal(self, location):
        goal_msg = Move.Goal()
        goal_msg.location = location

        self.get_logger().info(f'Sending goal to move to: {location}')
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {'Success' if result.success else 'Failed'}, Message: '{result.message}'")


def main(args=None):
    rclpy.init(args=args)

    action_client = MoveActionClient()

    test_location = "Table"  

    action_client.send_goal(test_location)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()