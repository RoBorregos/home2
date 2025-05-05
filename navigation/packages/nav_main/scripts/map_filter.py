#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
class SimpleMapServer(Node):
    def __init__(self):
        super().__init__('map_pub')
        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=1
        )
        self.map_original = self.create_subscription(
            OccupancyGrid,
            '/map_org',
            self.map_callback,
            10
        )
        self.map_mod = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile
        )

       
    def map_callback(self,map:OccupancyGrid):
        self.map_mod.publish(map)


    # def publish_map(self):
    #     self.map_msg.header.stamp = self.get_clock().now().to_msg()
    #     self.publisher.publish(self.map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()