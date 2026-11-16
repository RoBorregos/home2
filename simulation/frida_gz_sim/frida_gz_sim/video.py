"""Side-by-side overview + ZED video with a status banner, for the sim checks."""

import threading

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

OVERVIEW_TOPIC = "/sim/overview_camera/image"
ZED_TOPIC = "/zed/zed_node/rgb/image_rect_color"
FPS = 10
PANEL_H = 360


class VideoRecorder(Node):
    """Side-by-side overview + ZED video with a status banner, written at FPS."""

    def __init__(self, path: str):
        super().__init__("shelf_check_recorder")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._overview = None
        self._zed = None
        self.status = "starting"
        self.lines: list[str] = []
        self._writer = None
        self._closed = False
        self._path = path
        self.create_subscription(Image, OVERVIEW_TOPIC, self._overview_cb, 2)
        self.create_subscription(Image, ZED_TOPIC, self._zed_cb, 2)
        self.create_timer(1.0 / FPS, self._write)

    def _overview_cb(self, msg):
        with self._lock:
            self._overview = self._bridge.imgmsg_to_cv2(msg, "bgr8")

    def _zed_cb(self, msg):
        with self._lock:
            self._zed = self._bridge.imgmsg_to_cv2(msg, "bgr8")

    @staticmethod
    def _fit(image):
        if image is None:
            return np.zeros((PANEL_H, 480, 3), np.uint8)
        scale = PANEL_H / image.shape[0]
        return cv2.resize(image, (int(image.shape[1] * scale), PANEL_H))

    def _write(self):
        with self._lock:
            if self._closed:
                return
            left, right = self._fit(self._overview), self._fit(self._zed)
        frame = np.hstack([left, right])
        banner = np.full(
            (30 + 24 * max(1, len(self.lines)), frame.shape[1], 3), 30, np.uint8
        )
        cv2.putText(
            banner,
            self.status,
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )
        for i, line in enumerate(self.lines):
            cv2.putText(
                banner,
                line,
                (10, 48 + 24 * i),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )
        frame = np.vstack([frame, banner])
        if self._writer is None:
            self._size = (frame.shape[1], frame.shape[0])
            self._writer = cv2.VideoWriter(
                self._path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, self._size
            )
        if (frame.shape[1], frame.shape[0]) != self._size:
            frame = cv2.resize(frame, self._size)
        with self._lock:
            if not self._closed:
                self._writer.write(frame)

    def close(self):
        with self._lock:
            self._closed = True
            if self._writer is not None:
                self._writer.release()
