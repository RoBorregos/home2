#!/usr/bin/env python3

"""
Node to cleargrasp transforms
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge

# Paths
#case_name = sys.argv[1]
CONFIG_FILE_PATH = "config/config.yaml"
#RGB_PATH = f"../../../../cleargrasp/npy/rgb_image.npy"
RGB_PATH = "rgb_image.npy"
#DEPTH_PATH = f"../../../../cleargrasp/npy/input_depth.npy"
DEPTH_PATH = "input_depth.npy"
CAMERA_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEPTH_IMAGE_TOPIC = "/zed/zed_node/depth/depth_registered"

CLARGRASP_TOPIC = "/cleargrasp/pointcloud"

class cleargraspNode(Node):
    def __init__(self):
        super().__init__("cleargrasp_node")
        
        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10
        )

        self.depth_subscriber = self.create_subscription(
            PointCloud2, DEPTH_IMAGE_TOPIC, self.depth_callback, 10
        )
        self.get_logger().info(f"Subscribed to {CAMERA_TOPIC} and {DEPTH_IMAGE_TOPIC}")


        self.image = None
        self.output_image = None
        self.point_cloud_data = None
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(PointCloud2, CLARGRASP_TOPIC, 10)

    def image_callback(self, data):
        """Callback to receive the image from the camera."""
        try:
            self.get_logger().info('Received image')
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.image is None:
                return
            np.save(RGB_PATH, self.image)

        except Exception as e:
            print(f"Error: {e}")

    def depth_callback(self, msg: PointCloud2):
        try:
            print("Depth callback triggered")
            self.get_logger().info('Received PointCloud2')
            self.point_cloud_data = msg
            
            #points = self.convert_point_cloud_msg_to_numpy(self.point_cloud_data)
            #np.save(DEPTH_PATH, points)
            #print("Success convertion")
        
        except Exception as e:
            print(f"Error: {e}")
        
        
    def convert_point_cloud_msg_to_numpy(self, data: PointCloud2):
        if self.point_cloud_data is None:
            print("Empty point cloud")
            return np.array([])
            
        print('test message 1')
        # Parse the PointCloud2 message
        gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
                            
        print('test message 2')

        # Convert the point cloud to a numpy array
        points_numpy = np.array(list(gen), dtype=np.float32)

        return points_numpy


def main(args=None):
    print("Starting cleargrasp node...")
    rclpy.init(args=args)
    node = cleargraspNode()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()