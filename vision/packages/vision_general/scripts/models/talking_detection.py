#!/usr/bin/env python3
"""Talking detection from face landmarks: mouth-ratio oscillation plus debounce."""

from collections import deque

import numpy as np
from frida_constants.vision_constants import (
    LEFT_MOUTH_LANDMARK,
    LOWER_LIP_LANDMARK,
    RIGHT_MOUTH_LANDMARK,
    TALKING_DEBOUNCE_OFF_FRAMES,
    TALKING_DEBOUNCE_ON_FRAMES,
    TALKING_DIRECTION_CHANGE_THRESHOLD,
    TALKING_HISTORY_FRAMES,
    TALKING_MIN_DELTA,
    TALKING_MIN_MEAN_DELTA,
    TALKING_RATIO_CEILING,
    TALKING_SMOOTHING_WINDOW,
    UPPER_LIP_LANDMARK,
)


def get_mouth_ratio(landmarks) -> float:
    mouth_height = abs(
        landmarks[UPPER_LIP_LANDMARK].y - landmarks[LOWER_LIP_LANDMARK].y
    )
    mouth_width = abs(
        landmarks[LEFT_MOUTH_LANDMARK].x - landmarks[RIGHT_MOUTH_LANDMARK].x
    )
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


class MouthActivity:
    """Talking state for one person across a sequence of frames."""

    def __init__(self):
        self.ratio_buffer: deque = deque(maxlen=TALKING_HISTORY_FRAMES)
        self.raw_ratio_buffer: deque = deque(maxlen=TALKING_SMOOTHING_WINDOW)
        self.debounce_counter: int = 0
        self.confirmed_talking: bool = False

    def update(self, landmarks) -> bool:
        """Process one frame's landmarks and return whether the person is confirmed talking."""
        if landmarks is None:
            self.ratio_buffer.clear()
            self.raw_ratio_buffer.clear()
            self._step_debounce(False)
            return self.confirmed_talking

        self.raw_ratio_buffer.append(get_mouth_ratio(landmarks))
        self.ratio_buffer.append(float(np.mean(self.raw_ratio_buffer)))

        window = list(self.ratio_buffer)
        direction_changes = count_direction_changes(window, TALKING_MIN_DELTA)
        mean_ratio = float(np.mean(window))
        mean_delta = float(np.mean(np.abs(np.diff(window)))) if len(window) > 1 else 0.0

        raw_talking = (
            direction_changes >= TALKING_DIRECTION_CHANGE_THRESHOLD
            and mean_ratio < TALKING_RATIO_CEILING
            and mean_delta >= TALKING_MIN_MEAN_DELTA
        )
        self._step_debounce(raw_talking)
        return self.confirmed_talking

    def _step_debounce(self, raw_talking: bool) -> None:
        if raw_talking:
            self.debounce_counter = min(
                self.debounce_counter + 1, TALKING_DEBOUNCE_ON_FRAMES
            )
        else:
            self.debounce_counter = max(
                self.debounce_counter - 1, -TALKING_DEBOUNCE_OFF_FRAMES
            )

        if self.debounce_counter >= TALKING_DEBOUNCE_ON_FRAMES:
            self.confirmed_talking = True
        elif self.debounce_counter <= -TALKING_DEBOUNCE_OFF_FRAMES:
            self.confirmed_talking = False
