#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from frida_interfaces.srv import GraspDetection


class GraspDetectionClient(Node):
    def __init__(self):
        super().__init__("grasp_detection_client")
        self.client = self.create_client(GraspDetection, "detect_grasps")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        self.request = GraspDetection.Request()

    def send_request(self, cfg_path, pcd_path=None, cloud=None):
        self.request.cfg_path = cfg_path

        if pcd_path:
            self.request.pcd_path = pcd_path
        elif cloud:
            self.request.input_cloud = cloud
        else:
            self.get_logger().error(
                "Must provide either a PCD file path or a PointCloud2 message"
            )
            return None

        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    client = GraspDetectionClient()

    # Configure paths
    config_path = "/home/dominguez/roborregos/home_ws/src/manipulation/packages/gpd/cfg/eigen_params.cfg"
    pcd_path = "/home/dominguez/roborregos/home_ws/src/manipulation/packages/gpd/tutorials/krylon.pcd"

    response = client.send_request(config_path, pcd_path=pcd_path)

    if response is not None:
        if response.success:
            print(f"Successful detection! {len(response.grasp_poses)} grasps found")

            for i, (pose, score) in enumerate(
                zip(response.grasp_poses, response.grasp_scores)
            ):
                print(f"\nGrip {i + 1} - Score: {score:.3f}")
                print(
                    f"Position: [x: {pose.pose.position.x:.3f}, "
                    f"y: {pose.pose.position.y:.3f}, "
                    f"z: {pose.pose.position.z:.3f}]"
                )
                print(
                    f"Orientation: [x: {pose.pose.orientation.x:.3f}, "
                    f"y: {pose.pose.orientation.y:.3f}, "
                    f"z: {pose.pose.orientation.z:.3f}, "
                    f"w: {pose.pose.orientation.w:.3f}]"
                )
        else:
            print(f"Service error: {response.message}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
