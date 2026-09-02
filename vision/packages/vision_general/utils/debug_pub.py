"""Subscriber-gated, rate-limited debug image publisher.

Vision nodes publish annotated debug frames continuously (raw bgr8 Image at
10-30 Hz) whether or not anything consumes them — ~2.7 MB/frame at 720p paid in
cv_bridge conversion, memcpy and DDS writes. This helper makes that cost zero
when nothing is subscribed and ~20x smaller when something is:

- keeps the node's legacy raw Image topic (web_video_server streams it for the
  display) but only converts/publishes when that topic has subscribers;
- additionally offers `/vision/debug/<name>/compressed`
  (sensor_msgs/CompressedImage, JPEG) which web_video_server can serve as
  passthrough MJPEG via `?type=ros_compressed`;
- rate-limits both to `max_hz` (demos don't need 30 fps).

Usage (drop-in for the publish_image pattern):
    self.image_publisher = DebugImagePublisher(self, IMAGE_TOPIC_HRIC, "hric_commands")
    ...
    self.image_publisher.publish(self.output_image)
"""

import time

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

DEBUG_TOPIC_FMT = "/vision/debug/{name}/compressed"


class DebugImagePublisher:
    def __init__(
        self,
        node,
        legacy_topic: str,
        name: str,
        max_hz: float = 5.0,
        jpeg_quality: int = 70,
        callback_group=None,
    ):
        self._bridge = CvBridge()
        self._min_period = 1.0 / max_hz
        self._encode_params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        self._last_pub = 0.0
        kwargs = {"callback_group": callback_group} if callback_group else {}
        self._raw_pub = node.create_publisher(Image, legacy_topic, 10, **kwargs)
        self._jpg_pub = node.create_publisher(
            CompressedImage, DEBUG_TOPIC_FMT.format(name=name), 10, **kwargs
        )

    def has_subscribers(self) -> bool:
        """True when either the raw or the compressed debug topic is consumed
        (e.g. web_video_server streaming it for the display)."""
        return (
            self._raw_pub.get_subscription_count() > 0
            or self._jpg_pub.get_subscription_count() > 0
        )

    def publish(self, frame_bgr) -> None:
        """Publish a bgr8 debug frame; no-op unless someone is subscribed."""
        if frame_bgr is None or len(frame_bgr) == 0:
            return
        want_raw = self._raw_pub.get_subscription_count() > 0
        want_jpg = self._jpg_pub.get_subscription_count() > 0
        if not (want_raw or want_jpg):
            return
        now = time.monotonic()
        if now - self._last_pub < self._min_period:
            return
        self._last_pub = now

        if want_raw:
            self._raw_pub.publish(self._bridge.cv2_to_imgmsg(frame_bgr, "bgr8"))
        if want_jpg:
            ok, buf = cv2.imencode(".jpg", frame_bgr, self._encode_params)
            if ok:
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.data = buf.tobytes()
                self._jpg_pub.publish(msg)
