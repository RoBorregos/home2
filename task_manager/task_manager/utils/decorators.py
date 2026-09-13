"""
Decorators for subtask managers
"""

import functools
import time
from typing import Any, Callable, Optional, Union

from rclpy.action import ActionClient
import rclpy.client
from .logger import Logger
from .run_log import RunLog


def mockable(return_value=None, delay=0, mock=False, _mock_callback=None):
    """
    Decorator to return mock values instead of performing
    the function.
    Args:
        return_value: Value to return if mock_data is True
        delay: Delay in seconds before returning the value
    """

    def decorator(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            if getattr(self, "mock_data", False) or mock:
                if _mock_callback is not None:
                    return _mock_callback(self, **kwargs)
                if delay > 0:
                    time.sleep(delay)
                value = return_value(self) if callable(return_value) else return_value
                Logger.mock(self.node, f"{func.__name__}. Value: {value}")
                return value
            return func(self, *args, **kwargs)

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
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            service_client = getattr(self, client)

            value = return_value(self) if callable(return_value) else return_value

            if isinstance(service_client, rclpy.client.Client):
                if not service_client.wait_for_service(timeout_sec=timeout):
                    self.node.get_logger().error(f"Service not available for: {client}.")
                    return value

            elif isinstance(service_client, ActionClient):
                if not service_client.wait_for_server(timeout_sec=timeout):
                    self.node.get_logger().error(f"Action not available for: {client}.")
                    return value
            return func(self, *args, **kwargs)

        return wrapper

    return decorator


def measured(
    skill: Optional[str] = None,
    context: Optional[Union[dict, Callable[..., dict]]] = None,
):
    """
    Record the outcome and wall-clock duration of a skill call to the current run log.

    Feeds the capability manifest (measured success rate and p50 duration per skill),
    which the score-aware selector uses to decide what is worth attempting.
    Args:
        skill: Name to record under. Defaults to the wrapped function's name.
        context: Extra tags to bucket the measurement by (e.g. object class, surface).
            Either a plain dict or a callable receiving the same args as the function.
    """

    def decorator(func):
        name = skill or func.__name__

        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            # mocked calls would poison the manifest with fabricated durations
            if getattr(self, "mock_data", False):
                return func(self, *args, **kwargs)

            start = time.perf_counter()
            status = None
            outcome = None
            try:
                outcome = func(self, *args, **kwargs)
                status = _outcome_status(outcome)
                return outcome
            except Exception as error:
                status = f"EXCEPTION:{type(error).__name__}"
                raise
            finally:
                RunLog.record(
                    skill=name,
                    status=status,
                    duration_s=time.perf_counter() - start,
                    context=_resolve_context(context, self, args, kwargs),
                    result=outcome,
                )

        return wrapper

    return decorator


def _outcome_status(outcome: Any) -> Any:
    """Subtask managers return either a bare Status or a (Status, payload) tuple."""
    if isinstance(outcome, (tuple, list)) and outcome:
        return outcome[0]
    return outcome


def _resolve_context(context, instance, args, kwargs) -> dict:
    """Context tags must never break the skill they annotate."""
    if context is None:
        return {}
    if callable(context):
        try:
            return context(instance, *args, **kwargs) or {}
        except Exception:
            return {}
    return dict(context)
