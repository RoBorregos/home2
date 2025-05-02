#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class VirtualStop(Node):
    def __init__(self):
        super().__init__('check_door')

        self.range_max = 870
        self.range_min = 750
        self.closed_distance = 0.7

        self.joy_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.check_door,
            10
        )
        
        #self.publisher = self.create_publisher(LaserScan, '/filtered_2_scan', 10) # Debug

    def check_door(self,msg: LaserScan):
        # self.get_logger().info(f'Received message: {msg}')
        # filtered_msg = LaserScan()
        # filtered_msg.header = msg.header
        # filtered_msg.angle_min = msg.angle_min
        # filtered_msg.angle_max = msg.angle_max
        # filtered_msg.angle_increment = msg.angle_increment
        # filtered_msg.time_increment = msg.time_increment
        # filtered_msg.scan_time = msg.scan_time
        # filtered_msg.range_min = msg.range_min
        # filtered_msg.range_max = msg.range_max
        # filtered_msg.intensities = msg.intensities  #Debug

        door_points = []
        #filtered_ranges = []
        # length = len(msg.ranges)
        # self.get_logger().info(f'Length of ranges: {length}')
        for count, r in enumerate(msg.ranges):
            if  self.range_min <= count <= self.range_max:
                # filtered_ranges.append(r) # debug
                door_points.append(r)
            # else:
            #     filtered_ranges.append(float('nan'))  # mark as invalid
        
        # Check if the door is open
        if len(door_points) > 0:
            # Calculate the average distance of the door points
            avg_distance = sum(door_points) / len(door_points)
            # Check if the average distance is less than a threshold
            self.get_logger().info(f'Average distance: {avg_distance}')
            if avg_distance < self.closed_distance:
                self.get_logger().info(f'Door closed')
            else:
                self.get_logger().info('Door open')
        
        #filtered_msg.ranges = filtered_ranges
        # self.publisher.publish(filtered_msg)



def main(args=None):
    
    rclpy.init(args=args)
    move_action_server = VirtualStop()
    rclpy.spin(move_action_server)


if __name__ == '__main__':
    main()