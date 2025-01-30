import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import json

from frida_interfaces.navigation.action import Move

class MoveActionServer(Node):

    def __init__(self):
        super().__init__('move_action_server')

         # Cargar locaciones desde un archivo JSON
        with open('areas.json', 'r') as file:
            self.locations = json.load(file)

        self._action_server = ActionServer(
            self,
            Move,
            'Move',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Moving...')
        target_location = goal_handle.request.location

        # Validar si la locación existe en el JSON
        if not self.is_valid_location(target_location):
            self.get_logger().info(f"Invalid location: {target_location}")
            goal_handle.abort()
            result = Move.Result()
            result.success = False
            result.message = f"Location '{target_location}' not available."
            return result
        
        goal_handle.succeed()

        result = Move.Result()
        result.success = True
        result.message = f"Successfully moved to {target_location}"
        return result
    

    # Verificar si la locación solicitada existe en el JSON.
    def is_valid_location(self, location):
        for area, objects in self.locations.items():
            if location in objects:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = MoveActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()