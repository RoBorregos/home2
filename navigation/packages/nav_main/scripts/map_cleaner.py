#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2


class MapCleaner(Node):
    def __init__(self):
        super().__init__("map_cleaner")

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )
        qos2 = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.subscription = self.create_subscription(
            OccupancyGrid, "/map_input", self.map_callback,  qos
        )

        self.publisher = self.create_publisher(OccupancyGrid, "/map", qos2)

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Convert to binary (255 = occupied, 0 = free)
        binary = np.where(grid > 50, 255, 0).astype(np.uint8)

        cleaned_binary = cv2.morphologyEx(
            binary, cv2.MORPH_CLOSE, np.ones((2, 2), np.uint8), iterations=3
        )
        # Back to occupancy values
        cleaned = np.where(cleaned_binary == 255, 100, 0).astype(np.int8)
        cleaned[grid == -1] = -1  # preserve unknowns

        # Create and publish new map
        new_map = OccupancyGrid()
        new_map.header = msg.header
        new_map.info = msg.info
        new_map.data = cleaned.flatten().tolist()

        self.publisher.publish(new_map)
        self.get_logger().info("Published cleaned map.")


def main(args=None):
    rclpy.init(args=args)
    node = MapCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()