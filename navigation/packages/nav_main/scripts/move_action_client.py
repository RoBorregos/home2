#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from frida_interfaces.action import Move


class MoveActionClient(Node):

    def __init__(self):
        super().__init__('move_action_client')

        # Crear el cliente 
        self._action_client = ActionClient(self, Move, 'move')

    def send_goal(self, location):
        goal_msg = Move.Goal()
        goal_msg.location = location

        self.get_logger().info(f'Sending goal to move to: {location}')

        # Esperar que el servidor de acción esté disponible
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Server not available.")
            return

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal was rejected by the server.")
            return
        
        self.get_logger().info("Goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Result: {'Success' if result.success else 'Failed'}, Message: '{result.message}'")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveActionClient()

    while rclpy.ok():
        location = input("Enter a location: ")  # Pedir locación en la terminal
        if location.lower() == "exit":
            break  # Permite salir del loop escribiendo "exit"
        
        action_client.send_goal(location)

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
