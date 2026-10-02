#!/usr/bin/env python3
"""Talking detection from MediaPipe face landmarks and mouth-ratio oscillation."""

from collections import deque

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python.vision import (
    FaceLandmarker,
    FaceLandmarkerOptions,
    RunningMode,
)

UPPER_LIP = 13
LOWER_LIP = 14
LEFT_MOUTH = 61
RIGHT_MOUTH = 291

HISTORY_FRAMES = 25
MIN_DELTA = 0.01
DIRECTION_CHANGE_THRESHOLD = 2
DEBOUNCE_ON_FRAMES = 3
DEBOUNCE_OFF_FRAMES = 5
RATIO_CEILING = 0.3
MIN_MEAN_DELTA = 0.01
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


class TalkingDetector:
    def __init__(self, model_path: str):
        options = FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.IMAGE,
            num_faces=1,
            min_face_detection_confidence=0.5,
            min_face_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = FaceLandmarker.create_from_options(options)

        self.ratio_buffer: deque = deque(maxlen=HISTORY_FRAMES)
        self.raw_ratio_buffer: deque = deque(maxlen=SMOOTHING_WINDOW)
        self.debounce_counter: int = 0
        self.confirmed_talking: bool = False

    def detect_landmarks(self, frame_bgr):
        """Return the first face's landmarks, or None if no face is visible."""
        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB),
        )
        results = self.landmarker.detect(mp_image)
        return results.face_landmarks[0] if results.face_landmarks else None

    def update(self, frame_bgr) -> bool:
        """Process one frame and return whether the person is confirmed talking."""
        return self.update_landmarks(self.detect_landmarks(frame_bgr))

    def update_landmarks(self, landmarks) -> bool:
        if landmarks is None:
            # Drop history so a face that reappears starts from a clean window
            # instead of inheriting the previous face's mouth movement.
            self.ratio_buffer.clear()
            self.raw_ratio_buffer.clear()
            self._step_debounce(False)
            return self.confirmed_talking

        self.raw_ratio_buffer.append(get_mouth_ratio(landmarks))
        self.ratio_buffer.append(float(np.mean(self.raw_ratio_buffer)))

        window = list(self.ratio_buffer)
        direction_changes = count_direction_changes(window, MIN_DELTA)
        mean_ratio = float(np.mean(window))
        mean_delta = float(np.mean(np.abs(np.diff(window)))) if len(window) > 1 else 0.0

        raw_talking = (
            direction_changes >= DIRECTION_CHANGE_THRESHOLD
            and mean_ratio < RATIO_CEILING
            and mean_delta >= MIN_MEAN_DELTA
        )
        self._step_debounce(raw_talking)
        return self.confirmed_talking

    def reset(self) -> None:
        self.ratio_buffer.clear()
        self.raw_ratio_buffer.clear()
        self.debounce_counter = 0
        self.confirmed_talking = False

    def close(self) -> None:
        self.landmarker.close()

    def _step_debounce(self, raw_talking: bool) -> None:
        if raw_talking:
            self.debounce_counter = min(self.debounce_counter + 1, DEBOUNCE_ON_FRAMES)
        else:
            self.debounce_counter = max(self.debounce_counter - 1, -DEBOUNCE_OFF_FRAMES)

        if self.debounce_counter >= DEBOUNCE_ON_FRAMES:
            self.confirmed_talking = True
        elif self.debounce_counter <= -DEBOUNCE_OFF_FRAMES:
            self.confirmed_talking = False
