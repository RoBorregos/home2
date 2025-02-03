#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory
from frida_interfaces.action import Move

class MoveActionServer(Node):

    
    def __init__(self):
        super().__init__('move_action_server')

        # Obtener la ruta de instalación del paquete
        package_share_directory = get_package_share_directory('nav_main')
        json_path = os.path.join(package_share_directory, 'locations', 'areas.json')
     
        # Intentar cargar el archivo JSON
        try:
            with open(json_path, 'r') as file:
                self.locations = json.load(file)
            self.get_logger().info("JSON loaded sucessfully.")
        except Exception as e:
            print(f"[ERROR] Failed to load JSON file: {e}")
            self.locations = {}


        # Inicializar el servidor de acción
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        target_location = goal_handle.request.location

        # Validar si la locación existe en el JSON
        if not self.is_valid_location(target_location):
            self.get_logger().info(f"Invalid location: {target_location}")
            goal_handle.abort()
            result = Move.Result()
            result.success = False
            result.message = f"Location '{target_location}' not available."
            return result
        
        self.get_logger().info(f'Moving to {target_location}...')
        goal_handle.succeed()

        result = Move.Result()
        result.success = True
        result.message = f"Successfully moved to {target_location}"
        return result
    

    # Verificar si la locación solicitada existe en el JSON.
    def is_valid_location(self, location):
        for area, objects in self.locations.items():
            if location == area:
                return True
            if location in objects:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)

    move_action_server = MoveActionServer()

    rclpy.spin(move_action_server)


if __name__ == '__main__':
    main()
