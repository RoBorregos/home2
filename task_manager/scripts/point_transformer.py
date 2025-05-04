#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.srv import PointTransformation, ReturnAreas
import json
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped

POINT_TRANSFORMER_TOPIC = "/integration/point_transformer"
RETURN_AREAS_TOPIC = "/integration/return_areas"


class PointTransformer(Node):
    def __init__(self):
        super().__init__("point_transformer")
        # Create a callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.set_target_service = self.create_service(
            PointTransformation, POINT_TRANSFORMER_TOPIC, self.set_target_callback
        )
        self.return_areas = self.create_service(ReturnAreas, RETURN_AREAS_TOPIC, self.whereIam)

        self.get_logger().info("PointTransformer node has been started.")

    def set_target_callback(self, request, response):
        """Convert the object to height"""
        try:
            # First check if transform is available with timeout
            if not self.tf_buffer.can_transform(
                request.target_frame,
                request.point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0),
            ):
                self.get_logger().error(
                    self, "Transform from camera to base_link not available yet"
                )
                response.success = False
                response.message = "Transform from camera to base_link not available yet"
                return response

            transform_frame = self.tf_buffer.lookup_transform(
                request.target_frame, request.point.header.frame_id, rclpy.time.Time()
            )

            transformed_point = do_transform_point(request.point, transform_frame)
            response.success = True
            response.transformed_point = transformed_point
            return response
        except Exception as e:
            self.get_logger().error(self, f"Error converting to height: {e}")
            response.success = False
            response.message = "Error converting to height: {e}"
            return response

    def whereIam(self, request, response):
        # Create a tf2 buffer and listener
        try:
            # Wait for the transform to become available (with a timeout)
            buffer_ = self.tf_buffer.can_transform(
                "map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)
            )
            self.get_logger().info(f"DEBUGG ={buffer_}")
            rclpy.spin_once(self, timeout_sec=0.1)
            # Get the transform from base_link to map
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )

            # Extract the pose information
            posex = transform.transform.translation.x
            posey = transform.transform.translation.y

            self.get_logger().info(f"Robot's position in the map frame: x={posex}, y={posey}")

        except Exception as e:
            self.get_logger().error(f"Could not get transform: {str(e)}")
            response.location = None
            response.success = False
            return response
        package_share_directory = get_package_share_directory("frida_constants")
        file_path = os.path.join(package_share_directory, "map_areas/areas.json")
        mylocation = ""
        with open(file_path, "r") as file:
            self.areas = json.load(file)
        for area in self.areas:
            self.get_logger().info(f"area: {area}")
            if self.is_inside(posex, posey, self.areas[area]["polygon"]):
                mylocation = area
                break
        if mylocation == "":
            self.get_logger().warning("I dont know where I am")
            response.location = None
            response.success = False
            return response
        self.get_logger().info(f"I AM IN: {mylocation}")
        response.location = mylocation
        response.success = True
        return response

    def is_inside(self, x, y, polygon):
        inside = False
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]

            if (y1 > y) != (y2 > y):
                xinters = (y - y1) * (x2 - x1) / (y2 - y1 + 1e-10) + x1  # Avoid zero division
                if x < xinters:
                    inside = not inside
        return inside


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()

    # Use MultiThreadedExecutor to handle concurrent callbacks
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
