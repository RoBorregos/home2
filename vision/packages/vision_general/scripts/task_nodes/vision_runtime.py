"""Shared base for vision_general's per-task nodes.

Counterpart to object_detector_2d's `base_detector_node.py`: the camera
subscriptions, the ``/vision/<name>/active`` gate, camera-rotation tracking, a
TF buffer and an optional ``DebugImagePublisher``.

Subclasses opt in through the constructor flags and read ``self.image``,
``self.depth_image`` and ``self.camera_info`` exactly as they did when each node
owned its own subscriptions.
"""

import functools
import inspect
import traceback

import rclpy
import rclpy.qos
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from frida_constants.vision_constants import (
    CAMERA_INFO_TOPIC,
    CAMERA_ROTATION_TOPIC,
    DEPTH_IMAGE_TOPIC,
    IMAGE_ORIENTED_TOPIC,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int16

from utils.debug_pub import DebugImagePublisher

ACTIVE_TOPIC_FMT = "/vision/{name}/active"


class VisionRuntime(Node):
    """Base class for vision_general's per-task nodes."""

    def __init__(
        self,
        node_name: str,
        *,
        image_topic: str = IMAGE_ORIENTED_TOPIC,
        use_depth: bool = False,
        use_camera_info: bool = False,
        use_tf: bool = False,
        track_rotation: bool = False,
        debug_topic: str = None,
        debug_name: str = None,
        active_name: str = None,
        active_node: bool = True,
    ):
        super().__init__(node_name)

        self.bridge = CvBridge()
        self.callback_group = ReentrantCallbackGroup()

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.image = None
        self.image_header = None
        self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos,
            callback_group=self.callback_group,
        )

        self.depth_image = None
        if use_depth:
            self.create_subscription(
                Image,
                DEPTH_IMAGE_TOPIC,
                self._depth_callback,
                qos,
                callback_group=self.callback_group,
            )

        self.camera_info = None
        if use_camera_info:
            self.create_subscription(
                CameraInfo,
                CAMERA_INFO_TOPIC,
                self._camera_info_callback,
                qos,
                callback_group=self.callback_group,
            )

        self.rotation = 0
        if track_rotation:
            self.create_subscription(
                Int16,
                CAMERA_ROTATION_TOPIC,
                self._rotation_callback,
                10,
                callback_group=self.callback_group,
            )

        self.tf_buffer = None
        self.tf_listener = None
        if use_tf:
            self.tf_buffer = tf2_ros.Buffer()
            # Held only to keep it alive: if it is collected, /tf stops updating.
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.debug_publisher = None
        if debug_topic:
            self.debug_publisher = DebugImagePublisher(
                self,
                debug_topic,
                debug_name or node_name,
                callback_group=self.callback_group,
            )

        self.active = active_node
        if active_name:
            self.create_subscription(
                Bool,
                ACTIVE_TOPIC_FMT.format(name=active_name),
                self._active_callback,
                10,
                callback_group=self.callback_group,
            )

    # ------------------------------------------------------------------ callbacks

    def _image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"Image conversion error: {e}")
            return
        self.image_header = msg.header

    def _depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            self.get_logger().warn(f"Depth conversion error: {e}")

    def _camera_info_callback(self, msg):
        self.camera_info = msg

    def _rotation_callback(self, msg):
        value = int(msg.data) % 360
        if value != self.rotation:
            self.rotation = value
            self.get_logger().info(f"Camera rotation set to {self.rotation}")

    def _active_callback(self, msg):
        self.active = bool(msg.data)
        self.get_logger().info(
            f"{self.get_name()} {'active' if self.active else 'paused'}"
        )

    # ------------------------------------------------------------------ helpers

    def publish_debug(self, frame):
        if self.debug_publisher is not None:
            self.debug_publisher.publish(frame)


def safe_service_callback(callback):
    """Wrap a service callback (self, request, response) -> response so an
    unhandled exception logs and returns response (with success=False and,
    if present, a message) instead of propagating.

    spin() below already keeps the node itself running through any
    unhandled callback exception, but the one request whose callback raised
    still never gets a reply and its caller hangs until its own timeout -
    this decorator is what gives THAT specific caller a clean failure
    response instead. Works for both `def` and `async def` callbacks.
    """

    def _on_failure(self, response, exc):
        self.get_logger().error(f"{callback.__qualname__} failed: {exc}")
        if hasattr(response, "success"):
            response.success = False
        if hasattr(response, "message"):
            response.message = f"{callback.__qualname__} failed: {exc}"
        return response

    if inspect.iscoroutinefunction(callback):

        @functools.wraps(callback)
        async def async_wrapper(self, request, response):
            try:
                return await callback(self, request, response)
            except Exception as e:
                return _on_failure(self, response, e)

        return async_wrapper

    @functools.wraps(callback)
    def sync_wrapper(self, request, response):
        try:
            return callback(self, request, response)
        except Exception as e:
            return _on_failure(self, response, e)

    return sync_wrapper


def spin(node, threads: int = 4):
    """Run `node` on a MultiThreadedExecutor and shut down cleanly.

    Drives spin_once() manually instead of calling executor.spin() directly:
    MultiThreadedExecutor._spin_once_impl re-raises any unhandled callback
    exception on the thread that's spinning, so a single buggy callback
    would otherwise propagate all the way out and kill this node's entire
    process. Catching per-iteration keeps every other callback (and every
    other in-flight request) running; only the one request whose callback
    raised is left without a response.
    """
    executor = rclpy.executors.MultiThreadedExecutor(threads)
    executor.add_node(node)
    try:
        while rclpy.ok():
            try:
                executor.spin_once()
            except Exception:
                node.get_logger().error(
                    "Unhandled exception in a callback; node keeps running:\n"
                    f"{traceback.format_exc()}"
                )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
