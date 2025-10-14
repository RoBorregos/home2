#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion


class DockingPoseRecorder(Node):
    def __init__(self):
        super().__init__("docking_pose_recorder")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PoseStamped, "/door_handle_pose", self.handle_pose_callback, 10
        )

        self.relative_pose_saved = False
        self.get_logger().info(
            "DockingPoseRecorder ready. Waiting for /door_handle_pose..."
        )

    def handle_pose_callback(self, handle_pose: PoseStamped):
        if self.relative_pose_saved:
            return

        try:
            # Lookup the transform from the robot to the door handle
            if not self.tf_buffer.can_transform(
                "base_link",
                handle_pose.header.frame_id,
                handle_pose.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0),
            ):
                self.get_logger().warn(
                    "Transform from base_link to door handle not available."
                )
                return
            transform = self.tf_buffer.lookup_transform(
                "base_link", handle_pose.header.frame_id, handle_pose.header.stamp
            )
            self.get_logger().info(
                f"Transform from {handle_pose.header.frame_id} to base_link found."
            )
            # since we are using base_link as the reference frame, we can directly use the handle_pose and use the robots as 000

            dx, dy, dz = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )

            # get the base_links orientation in reference to map
            t2 = self.tf_buffer.lookup_transform(
                "map", "base_link", handle_pose.header.stamp
            )
            q = t2.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            rel_pose = {
                "x": dx,
                "y": dy,
                "z": dz,
                "yaw": yaw,
            }

            with open("relative_docking_pose.yaml", "w") as f:
                yaml.dump(rel_pose, f)

            self.get_logger().info("Relative pose saved:")
            self.get_logger().info(str(rel_pose))
            self.relative_pose_saved = True

        except Exception as e:
            self.get_logger().warn(f"TF lookup or processing failed: {e}")


def main(args=None):
    print("Starting DockingPoseRecorder...")
    rclpy.init(args=args)
    node = DockingPoseRecorder()
    print("DockingPoseRecorder initialized. Waiting for /door_handle_pose...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
