#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, Pose
from tf2_geometry_msgs import do_transform_pose

# import tf2_py
from tf_transformations import euler_from_quaternion
# from builtin_interfaces.msg import Time


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
            # Lookup transform from base_link to map
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )

            # Transform base_link pose into map frame (get robot pose)
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = "base_link"
            robot_pose.pose.orientation.w = 1.0
            robot_pose.pose.position.x = 0.0
            robot_pose.pose.position.y = 0.0
            robot_pose.pose.position.z = 0.0
            rob_pose = Pose()
            rob_pose.pose = robot_pose.pose
            rob_pose = do_transform_pose(
                rob_pose, transform
            )  # Transform robot pose to map frame

            robot_pose.pose = rob_pose
            # Compute relative position (handle - robot)
            dx = handle_pose.pose.position.x - robot_pose.pose.position.x
            dy = handle_pose.pose.position.y - robot_pose.pose.position.y
            dz = handle_pose.pose.position.z - robot_pose.pose.position.z

            # Compute yaw difference
            q_handle = [
                handle_pose.pose.orientation.x,
                handle_pose.pose.orientation.y,
                handle_pose.pose.orientation.z,
                handle_pose.pose.orientation.w,
            ]
            q_robot = [
                robot_pose.pose.orientation.x,
                robot_pose.pose.orientation.y,
                robot_pose.pose.orientation.z,
                robot_pose.pose.orientation.w,
            ]

            # ...
            _, _, yaw_handle = euler_from_quaternion(q_handle)
            _, _, yaw_robot = euler_from_quaternion(q_robot)

            dyaw = yaw_handle - yaw_robot

            # Save to file or print
            rel_pose = {"x": dx, "y": dy, "z": dz, "yaw": dyaw}

            with open("relative_docking_pose.yaml", "w") as f:
                yaml.dump(rel_pose, f)

            self.get_logger().info("Relative pose saved:")
            self.get_logger().info(str(rel_pose))
            self.relative_pose_saved = True

        except Exception as e:
            self.get_logger().warn(f"TF lookup or processing failed: {e}")


print("DockingPoseRecorder initialized. Waiting for /door_handle_pose...")


def main(args=None):
    print("Starting DockingPoseRecorder...")
    rclpy.init(args=args)
    node = DockingPoseRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
