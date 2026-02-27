#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class SimulatePosition(Node):
    def __init__(self):
        super().__init__('simulateposition')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        baseData = PoseStamped()
        baseData.pose.position.x = 0.0
        baseData.pose.position.y = 0.0
        baseData.pose.position.z = 0.0
        baseData.pose.orientation.x = 0.0
        baseData.pose.orientation.y = 0.0
        baseData.pose.orientation.z = 0.0
        baseData.pose.orientation.w = 1.0

        self.make_transforms(baseData)

        self.subscription = self.create_subscription(PoseStamped,'/goal_pose',self.make_transforms,10)

    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link' 

        t.transform.translation.x = transformation.pose.position.x
        t.transform.translation.y = transformation.pose.position.y 
        t.transform.translation.z = transformation.pose.position.z 
        t.transform.rotation.x =  transformation.pose.orientation.x
        t.transform.rotation.y =  transformation.pose.orientation.y 
        t.transform.rotation.z =  transformation.pose.orientation.z 
        t.transform.rotation.w =  transformation.pose.orientation.w 

        self.tf_static_broadcaster.sendTransform(t)
            


def main(args=None):
    rclpy.init(args=args)
    simPosnode = SimulatePosition()
    rclpy.spin(simPosnode)


if __name__ == '__main__':
    main()
