#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import yaml
import numpy as np
from PIL import Image
import os
from ament_index_python.packages import get_package_share_directory

class SimpleMapServer(Node):
    def __init__(self):
        super().__init__('simple_map_server')
        package_share_dir = get_package_share_directory('nav_main')
        default_map_path = os.path.join(package_share_dir, 'maps', 'map_afuera.yaml')
        self.declare_parameter('map_yaml_file', default_map_path)

        # Declare and get the map YAML path
        yaml_path = self.get_parameter('map_yaml_file').get_parameter_value().string_value

        self.map_msg = self.load_map_from_yaml(yaml_path)
        self.publisher = self.create_publisher(OccupancyGrid, '/map_view', 10)

        # Publish at 1Hz
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info(f'Publishing map from: {yaml_path}')

    def load_map_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)

        image_path = os.path.join(os.path.dirname(yaml_path), map_data['image'])
        resolution = map_data['resolution']
        origin = map_data['origin']
        negate = map_data.get('negate', 0)
        occupied_thresh = map_data.get('occupied_thresh', 0.65)
        free_thresh = map_data.get('free_thresh', 0.25)

        # Load image
        img = Image.open(image_path).convert('L')  # Convert to grayscale
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        img_array = np.array(img)

        if negate:
            img_array = 255 - img_array

        # Create occupancy values
        data = []
        for pixel in img_array.flatten():
            occ = 100 if pixel / 255.0 < occupied_thresh else 0
            if pixel / 255.0 > free_thresh and pixel / 255.0 < occupied_thresh:
                occ = -1
            data.append(occ)

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = resolution
        grid.info.width = img.width
        grid.info.height = img.height
        grid.info.origin.position.x = origin[0]
        grid.info.origin.position.y = origin[1]
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0  # No rotation
        grid.data = data

        return grid

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)

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