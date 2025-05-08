#!/usr/bin/env python3

from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import rclpy

class RemovePerson(Node):
    """
    Receives a Point Stamped (human back position) from the topic used in follow person.
    """

    def __init__(self):
        super().__init__('remove_person')

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin_x = -1
        self.map_origin_y = -1

        self.current_person_pos = (-1, -1)
        self.last_person_pos = (-1, -1)
        self.map_data = []
        self.person_positions = []

        self.executing_goal = False
        self.robot_position = None
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.get_logger().info("Initializing")

        self.map_sub = self.create_subscription(
           OccupancyGrid, '/map_org',  self.map_callback,10
        )

        self.modified_map_publisher = self.create_publisher(
            OccupancyGrid,"/map", qos_profile=qos_profile
        )

        self.pose_subscriber = self.create_subscription(
           PointStamped, 'point_visualize', self.pose_callback, 10
        )

        self.get_logger().info("Remove Person Ready")

    def pose_callback(self, data: PointStamped):
        self.last_person_pos = (data.point.x, data.point.y)
        self.person_positions.append(self.last_person_pos)

    def get_area(self, width, center_row, center_col):
        print(
            center_row - width // 2,
            center_row + width // 2,
            center_col - width // 2,
            center_col + width // 2,
        )
        return (
            int(max(center_row - width // 2, 0)),
            int(min(center_row + width // 2, self.map_height)),
            int(max(center_col - width // 2, 0)),
            int(min(center_col + width // 2, self.map_width)),
        )

    def set_area_to_value(
        self, map: OccupancyGrid, value, start_row, end_row, start_col, end_col
    ):
        print(start_row, end_row, start_col, end_col)
        for row in range(start_row, end_row):
            for col in range(start_col, end_col):
                self.modify_map(map, row, col, value)

    def modify_map(self, map: OccupancyGrid, row, col, value):
        index = self.map_width * row + col
        
        self.map_data[index] = value

    def publish_map(self, map: OccupancyGrid):
        map.data = tuple(self.map_data)
        self.modified_map_publisher.publish(map)

    def map_callback(self, map: OccupancyGrid):
        self.map_data = list(map.data)
        self.map_width = map.info.width
        self.map_height = map.info.height
        self.map_resolution = map.info.resolution
        self.map_origin_x = map.info.origin.position.x
        self.map_origin_y = map.info.origin.position.y

        # if self.last_person_pos != (-1, -1):
        #     print("removing last person pos")
        #     self.set_area_to_value(map, -1, *self.get_area(15, *self.last_person_pos)) # check resolution

        for person_pos in self.person_positions:
            self.set_area_to_value(map, -1, *self.get_area(50, *person_pos)) # check resolution

        self.last_person_pos = (-1, -1)

        self.publish_map(map)

def main(args=None):
    rclpy.init(args=args)
    node = RemovePerson()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()