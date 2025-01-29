#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from frida_interfaces.srv import ItemsCategory

class Embeddings(Node):
    def __init__(self):  # Fixed typo here
        super().__init__('embeddings')
        self.srv = self.create_service(ItemsCategory, 'item_categorization', self.item_categorization_callback)

    def item_categorization_callback(self, request, response):
        # Here we would do the actual item categorization
        self.get_logger().info('Item categorization request received')
        response.category = 'some_category'  # You can customize this response
        return response
    
def main():
    rclpy.init()
    embeddings = Embeddings()
    rclpy.spin(embeddings)

    rclpy.shutdown()

if __name__ == '__main__':  # Fixed typo here
    main()
