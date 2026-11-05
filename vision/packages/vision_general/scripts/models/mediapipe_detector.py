#!/usr/bin/env python3
"""MediaPipe face landmarker wrapper. Loads the model and returns raw landmarks."""

import os
import pathlib

import cv2
import mediapipe as mp
from ament_index_python.packages import get_package_prefix
from frida_constants.vision_constants import FACE_LANDMARKER_MODEL
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python.vision import (
    FaceLandmarker,
    FaceLandmarkerOptions,
    RunningMode,
)


def resolve_model_path() -> pathlib.Path:
    """Prefer the persistent cache populated by fetch_models.py, else the packaged copy."""
    cache = (
        pathlib.Path(os.environ.get("TENSORRT_CACHE_DIR", "/workspace/trt_cache"))
        / FACE_LANDMARKER_MODEL
    )
    if cache.exists():
        return cache
    return (
        pathlib.Path(get_package_prefix("vision_general"))
        / f"lib/vision_general/models/{FACE_LANDMARKER_MODEL}"
    )


class MediapipeDetector:
    def __init__(self, model_path: str | None = None, num_faces: int = 1):
        model_path = model_path or str(resolve_model_path())
        options = FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.IMAGE,
            num_faces=num_faces,
            min_face_detection_confidence=0.5,
            min_face_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = FaceLandmarker.create_from_options(options)

    def detect(self, frame_bgr) -> list:
        """Return one landmark list per detected face (empty if no face is visible)."""
        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB),
        )
        return self.landmarker.detect(mp_image).face_landmarks

    def close(self) -> None:
        self.landmarker.close()
