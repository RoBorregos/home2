"""Person tracking model: YOLO + ByteTrack + ReID — no ROS dependencies.

Pose keypoints come from the shared DetectPose service (ObjectDetect2D): the
tracker node builds the
PoseDetection client (it needs a ROS node) and hands it to load().
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2
import torch
import torch.nn as nn
from PIL import Image as PILImage

from utils.models.reid_model import (
    extract_feature_from_img,
    get_structure,
    load_network,
)
from utils.trt_utils import load_yolo_trt

if TYPE_CHECKING:  # the client imports ROS; keep this module ROS-free
    from .pose_detection import PoseDetection

CONF_THRESHOLD = 0.6


class TrackerModel:
    """Bundles YOLO detection, ByteTrack tracking, pose estimation and ReID."""

    def __init__(self):
        self.yolo = None
        self.pose = None
        # Heavy SWIN ReID: only needed for re-acquisition after a lost track, so
        # it is loaded on demand through load_reid() (the tracker node does it on
        # a background thread) instead of at startup.
        self.reid = None

    def load(self, pose: PoseDetection) -> None:
        """Load the per-frame YOLO model and keep the pose client.

        ``pose`` is the DetectPose client built by the node; the pose model
        itself runs in ObjectDetect2D. ReID is loaded separately.
        """
        self.yolo = load_yolo_trt("yolov8n.pt")
        self.pose = pose

    def load_reid(self) -> None:
        """Build + load the SWIN ReID model. Takes several seconds — never call
        this from a callback that must stay responsive."""
        model = load_network(get_structure())
        model.classifier.classifier = nn.Sequential()
        model.eval()
        if torch.cuda.is_available():
            model = model.cuda()
        self.reid = model

    def track(self, frame, conf_threshold: float = CONF_THRESHOLD) -> list[dict]:
        """Per-frame multi-object tracking via ultralytics ByteTrack.

        Appearance-free: no per-detection ReID embedding. persist=True keeps the
        ByteTrack state (and thus the track-ids) across calls. Returns a list of
        {"track_id", "x1", "y1", "x2", "y2"} dicts (person class only).
        """
        frame_h, frame_w = frame.shape[:2]
        results = self.yolo.track(
            frame,
            classes=0,
            persist=True,
            tracker="bytetrack.yaml",
            verbose=False,
        )
        tracked = []
        if not results:
            return tracked
        boxes = results[0].boxes
        if boxes is None or boxes.id is None:
            return tracked
        ids = boxes.id.int().cpu().tolist()
        xyxy = boxes.xyxy.cpu().tolist()
        confs = boxes.conf.cpu().tolist()
        for tid, (bx1, by1, bx2, by2), conf in zip(ids, xyxy, confs):
            if conf < conf_threshold:
                continue
            x1 = max(0, min(int(round(bx1)), frame_w - 1))
            y1 = max(0, min(int(round(by1)), frame_h - 1))
            x2 = max(0, min(int(round(bx2)), frame_w))
            y2 = max(0, min(int(round(by2)), frame_h))
            if x2 <= x1 or y2 <= y1:
                continue
            tracked.append(
                {"track_id": int(tid), "x1": x1, "y1": y1, "x2": x2, "y2": y2}
            )
        return tracked

    def embed(self, crop_bgr):
        """Normalized ReID embedding (1-D tensor) for a BGR person crop."""
        pil = PILImage.fromarray(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB))
        with torch.no_grad():
            feat = extract_feature_from_img(pil, self.reid)
        return feat.flatten().float()
