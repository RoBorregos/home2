"""
Decorators for subtask managers
"""

import time
from rclpy.action import ActionClient
import rclpy.client


def mockable(return_value=None, delay=0):
    """
    Decorator to return mock values instead of performing
    the function.
    Args:
        return_value: Value to return if mock_data is True
        delay: Delay in seconds before returning the value
    """

    def decorator(func):
        def wrapper(self, *args, **kwargs):
            if delay > 0:
                time.sleep(delay)
            if getattr(self, "mock_data", False):
                return return_value

            func(self, *args, **kwargs)

        return wrapper

    return decorator


def service_check(client, return_value=None, timeout=3.0):
    """
    Check if the service is available before calling the
    function and return a default value if not.
    Args:
        client: Name of the client to check (service or action service)
        return_value: Value to return if service is not available
        timeout: Timeout in seconds to wait for the service
    """

    def decorator(func):
        def wrapper(self, *args, **kwargs):
            service_client = getattr(self, client)

            if isinstance(service_client, rclpy.client.Client):
                if not service_client.wait_for_service(timeout_sec=timeout):
                    self.node.get_logger().error(
                        f"Service not available for: {client}."
                    )
                    return return_value

            elif isinstance(service_client, ActionClient):
                if not service_client.wait_for_server(timeout_sec=timeout):
                    self.node.get_logger().error(f"Action not available for: {client}.")
                    return return_value
            return func(self, *args, **kwargs)

        return wrapper

    return decorator
