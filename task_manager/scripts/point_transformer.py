#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.callback_groups import ReentrantCallbackGroup
from frida_interfaces.srv import PointTransformation


POINT_TRANSFORMER_TOPIC = "/integration/point_transformer"


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
