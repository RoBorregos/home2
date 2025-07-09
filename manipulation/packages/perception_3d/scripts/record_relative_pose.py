#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped

# , Pose
from tf2_geometry_msgs import do_transform_pose

# import tf2_py
from tf_transformations import euler_from_quaternion
# , quaternion_from_euler
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
            # Transform the handle pose from its frame to the base_link frame
            transform = self.tf_buffer.lookup_transform(
                "map",  # target frame
                handle_pose.header.frame_id,  # source frame
                handle_pose.header.stamp,  # time of the pose
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            # Apply the transform to get the handle pose in base_link frame
            handle_in_base_link = do_transform_pose(handle_pose.pose, transform)

            # The handle_in_base_link now contains the position and orientation
            # of the handle relative to base_link
            dx = handle_in_base_link.position.x
            dy = handle_in_base_link.position.y
            dz = handle_in_base_link.position.z

            # Extract orientation as Euler angles
            q_handle = [
                handle_in_base_link.orientation.x,
                handle_in_base_link.orientation.y,
                handle_in_base_link.orientation.z,
                handle_in_base_link.orientation.w,
            ]

            self.get_logger().info(f"Handle orientation (quaternion): {q_handle}")
            self.get_logger().info(
                f"Handle position in base_link: x={dx}, y={dy}, z={dz}"
            )

            _, _, yaw_handle = euler_from_quaternion(q_handle)
            self.get_logger().info(f"Handle yaw in base_link: {yaw_handle}")
            # Since we're already in base_link frame, the robot's orientation is (0,0,0)
            yaw_robot = 0.0

            dyaw = yaw_handle - yaw_robot

            # Save to file or print
            rel_pose = {
                "x": -1 * float(dx),
                "y": -1 * float(dy),
                "z": -1 * float(dz),
                "yaw": float(dyaw),
            }
            self.get_logger().warn(
                f"Relative pose: {rel_pose} not handling yaw correction yet lol"
            )
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
