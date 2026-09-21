#!/usr/bin/env python3

"""
Node to detect whether a person is currently talking using MediaPipe face landmarks
and mouth ratio oscillation analysis.

Subscribes to the camera topic and continuously processes frames.
Exposes a service that returns whether a person is currently talking.

--- Run the node ---
    ros2 run vision_general talking_detection

--- Query the service ---
    ros2 service call /vision/is_talking std_srvs/srv/Trigger
"""

import pathlib
from collections import deque

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python.vision import (
    FaceLandmarker,
    FaceLandmarkerOptions,
    RunningMode,
)

import rclpy
import rclpy.qos
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from frida_constants.vision_constants import (
    CAMERA_TOPIC,
    IS_TALKING_TOPIC,
)

# Inner lip landmarks (MediaPipe 478-point mesh)
UPPER_LIP = 13
LOWER_LIP = 14
LEFT_MOUTH = 61
RIGHT_MOUTH = 291

# Tuned against ~4100 labeled frames across 3 recorded sessions (see the
# talking-detection tuning project) to minimize false positives on a held-open,
# non-talking mouth: 3.3% FP rate on "open" spans, 4.2% on silent, at 60% recall.
HISTORY_FRAMES = 25
MIN_DELTA = 0.01
DIRECTION_CHANGE_THRESHOLD = 2
DEBOUNCE_ON_FRAMES = 3
DEBOUNCE_OFF_FRAMES = 5
RATIO_CEILING = 0.3
MIN_MEAN_DELTA = 0.01
# Raw mouth-ratio landmarks jitter frame-to-frame even when the mouth is closed
# and still, which made direction-change counting mostly measure noise rather
# than real oscillation. Averaging the last N raw ratios before it enters the
# history window fixed that (dropped silent-frame false positives from ~36% to
# ~4% in tuning).
SMOOTHING_WINDOW = 5


def get_mouth_ratio(landmarks) -> float:
    mouth_height = abs(landmarks[UPPER_LIP].y - landmarks[LOWER_LIP].y)
    mouth_width = abs(landmarks[LEFT_MOUTH].x - landmarks[RIGHT_MOUTH].x)
    if mouth_width == 0:
        return 0.0
    return mouth_height / mouth_width


def count_direction_changes(buffer, min_delta: float) -> int:
    if len(buffer) < 3:
        return 0
    significant_diffs = []
    for i in range(1, len(buffer)):
        d = buffer[i] - buffer[i - 1]
        if abs(d) >= min_delta:
            significant_diffs.append(d)
    changes = 0
    for i in range(1, len(significant_diffs)):
        if significant_diffs[i - 1] * significant_diffs[i] < 0:
            changes += 1
    return changes


class TalkingDetectionNode(Node):
    def __init__(self):
        super().__init__("talking_detection")
        self.get_logger().info("Talking detection node initializing")
        self.bridge = CvBridge()

        MODELS_PATH = (
            pathlib.Path(get_package_share_directory("vision_general"))
            / "Utils"
            / "models"
        )
        model_path = str(MODELS_PATH / "face_landmarker.task")

        options = FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.IMAGE,
            num_faces=1,
            min_face_detection_confidence=0.5,
            min_face_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.detector = FaceLandmarker.create_from_options(options)
        self.get_logger().info("Face landmarker model loaded")

        # Detection state — maintained across frames
        self.ratio_buffer: deque = deque(maxlen=HISTORY_FRAMES)
        self.raw_ratio_buffer: deque = deque(maxlen=SMOOTHING_WINDOW)
        self.debounce_counter: int = 0
        self.confirmed_talking: bool = False

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        self.image_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos
        )

        self.is_talking_service = self.create_service(
            Trigger, IS_TALKING_TOPIC, self.is_talking_callback
        )

        self.get_logger().info("Talking detection node ready")

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
        )
        results = self.detector.detect(mp_image)

        if not results.face_landmarks:
            # No face detected — treat as silent, decay debounce toward off
            self.debounce_counter = max(self.debounce_counter - 1, -DEBOUNCE_OFF_FRAMES)
            if self.debounce_counter <= -DEBOUNCE_OFF_FRAMES:
                self.confirmed_talking = False
            return

        landmarks = results.face_landmarks[0]
        ratio = get_mouth_ratio(landmarks)
        self.raw_ratio_buffer.append(ratio)
        smoothed_ratio = float(np.mean(self.raw_ratio_buffer))
        self.ratio_buffer.append(smoothed_ratio)

        window = list(self.ratio_buffer)
        direction_changes = count_direction_changes(window, MIN_DELTA)
        mean_ratio = float(np.mean(window))
        mean_delta = float(np.mean(np.abs(np.diff(window)))) if len(window) > 1 else 0.0

        raw_talking = (
            direction_changes >= DIRECTION_CHANGE_THRESHOLD
            and mean_ratio < RATIO_CEILING
            and mean_delta >= MIN_MEAN_DELTA
        )

        if raw_talking:
            self.debounce_counter = min(self.debounce_counter + 1, DEBOUNCE_ON_FRAMES)
        else:
            self.debounce_counter = max(self.debounce_counter - 1, -DEBOUNCE_OFF_FRAMES)

        if self.debounce_counter >= DEBOUNCE_ON_FRAMES:
            self.confirmed_talking = True
        elif self.debounce_counter <= -DEBOUNCE_OFF_FRAMES:
            self.confirmed_talking = False

    def is_talking_callback(
        self, request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = self.confirmed_talking
        response.message = "TALKING" if self.confirmed_talking else "SILENT"
        self.get_logger().info(f"Is talking query: {response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TalkingDetectionNode()

    try:
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.detector.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
