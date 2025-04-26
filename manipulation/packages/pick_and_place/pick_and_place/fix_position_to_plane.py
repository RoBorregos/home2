import rclpy

from frida_interfaces.srv import GetPlaneBbox


class FixPositionToPlane(rclpy.Node):
    def __init__(self):
        super().__init__("fix_position_to_plane")
        self.get_plane_bbox_client = self.create_client(GetPlaneBbox, "get_plane_bbox")
        while not self.get_plane_bbox_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "get_plane_bbox service not available, waiting again..."
            )
        self.get_logger().info("get_plane_bbox service is available")

    def get_plane_bbox(self):
        req = GetPlaneBbox.Request()
        future = self.get_plane_bbox_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            raise Exception("Service call failed")
        # self.get_plane_bbox_client.call_async(req).result()

    # def
