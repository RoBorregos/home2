#!/usr/bin/env python3
import os
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
class PlyPublisher(Node):
    def __init__(self):
        super().__init__("ply_publisher")

        # PointCloud2 Publisher
        self.publisher_ = self.create_publisher(PointCloud2, "/pointcloud_topic", 10)
        self.get_logger().info("PlyPublisher has started.")

        # Read PLY with Open3D
        script_dir = os.path.dirname(os.path.realpath(__file__))
        package_share = get_package_share_directory('perception_3d')
        ply_path = os.path.join(package_share, "pointcloud.ply")
        #ply_path = os.path.join(script_dir, "pino.ply")
        pcd = o3d.io.read_point_cloud(ply_path)
        self.points = np.asarray(pcd.points, dtype=np.float32)

        # Timer 
        self.timer = self.create_timer(0.1, self.publish_cloud)  # 10 Hz

    def publish_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        points_list = [tuple(p) for p in self.points]

        cloud_msg = pc2.create_cloud(
            header,
            [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            points_list,
        )

        self.publisher_.publish(cloud_msg)
        self.get_logger().info(f"Published point cloud with {len(self.points)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = PlyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
