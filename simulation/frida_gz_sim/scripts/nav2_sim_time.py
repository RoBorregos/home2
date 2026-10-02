#!/usr/bin/env python3
"""Puts nav2's lifecycle manager on sim time before nav_central starts it.

nav2_omni.launch.py hardcodes use_sim_time: False on lifecycle_manager_navigation,
and a node's own parameters win over the launch-wide SetParameter. Left on wall
time it measures the servers' bond heartbeats against the wrong clock and drops
them whenever the sim is not running at 1.0 real-time factor.
"""

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node


class Nav2SimTime(Node):
    def __init__(self):
        super().__init__("nav2_sim_time")
        target = self.declare_parameter("target", "/lifecycle_manager_navigation").value
        timeout = self.declare_parameter("timeout", 180.0).value
        client = self.create_client(SetParameters, f"{target}/set_parameters")
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"{target} never appeared")
            raise SystemExit(1)
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="use_sim_time",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=True
                ),
            )
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        result = future.result()
        if result is None or not all(r.successful for r in result.results):
            self.get_logger().error(f"{target} refused use_sim_time")
            raise SystemExit(1)
        self.get_logger().info(f"{target} switched to sim time")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Nav2SimTime()
        node.destroy_node()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
