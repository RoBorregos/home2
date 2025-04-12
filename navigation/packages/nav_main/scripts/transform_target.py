#!/usr/bin/env python3

# map → odom → base_link → camera_link

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped
import tf2_ros
import tf2_geometry_msgs

from frida_constants.vision_constants import (
    RESULTS_TOPIC,
)

from frida_constants.navigation_constants import (
    FOLLOW_PERSON_TOPIC,
)

class TransformTarget(Node):
    def __init__(self):
        super().__init__('transform_target_node')

        # Escucha las coordenadas en puntos 3D
        self.subscription = self.create_subscription(
            Point,
            RESULTS_TOPIC,  
            self.point_callback,
            10
        )

        # Publica las coordenadas transformadas
        self.transformed_pub = self.create_publisher(
            PointStamped,
            FOLLOW_PERSON_TOPIC,
            10
        )

        # Buffer y listener TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def point_callback(self, msg: Point):
        try:
            # Arma un PointStamped con frame_id del sensor 
            point_stamped = PointStamped()
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.header.frame_id = 'camera_link'  # checar frame activo
            point_stamped.point = msg

            # Transforma al frame de mapa
            transform = self.tf_buffer.lookup_transform(
                'map',  # target frame
                point_stamped.header.frame_id,  # source frame
                rclpy.time.Time()
            )

            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)

            # Publica el punto transformado
            self.transformed_pub.publish(transformed_point)
            self.get_logger().info(f'Transformed point: {transformed_point.point}')

        except Exception as e:
            self.get_logger().warn(f'Failed to transform point: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TransformTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
