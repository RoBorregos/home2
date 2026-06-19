"""Person tracking model: YOLO + DeepSORT + ReID + PoseDetection — no ROS dependencies."""

import numpy as np
import torch
import torch.nn as nn
from PIL import Image as PILImage

from vision_general.utils.reid_model import (
    compare_images,
    compare_images_batch,
    extract_feature_from_img,
    extract_feature_from_img_batch,
    get_structure,
    load_network,
)
from vision_general.utils.deep_sort.detection import Detection as DeepSORTDetection
from vision_general.utils.deep_sort.nn_matching import NearestNeighborDistanceMetric
from vision_general.utils.deep_sort.tracker import Tracker as DeepSORTTracker
from vision_general.utils.trt_utils import load_yolo_trt
from .pose_detection import PoseDetection

CONF_THRESHOLD = 0.6
DEEPSORT_MAX_COSINE_DISTANCE = 0.3
DEEPSORT_NN_BUDGET = 100
DEEPSORT_MAX_AGE = 100
DEEPSORT_N_INIT = 3


class TrackerModel:
    """Bundles YOLO detection, DeepSORT tracking, ReID, and pose estimation."""

    def __init__(self):
        self.yolo = None
        self.pose = None
        self.reid = None
        self.deepsort = None

    def load(self) -> None:
        self.yolo = load_yolo_trt("yolov8n.pt")
        self.pose = PoseDetection()
        structure = get_structure()
        self.reid = load_network(structure)
        self.reid.classifier.classifier = nn.Sequential()
        if torch.cuda.is_available():
            self.reid = self.reid.cuda()
        metric = NearestNeighborDistanceMetric(
            "cosine", DEEPSORT_MAX_COSINE_DISTANCE, DEEPSORT_NN_BUDGET
        )
        self.deepsort = DeepSORTTracker(
            metric, max_age=DEEPSORT_MAX_AGE, n_init=DEEPSORT_N_INIT
        )

    def predict(self, frame):
        """Run YOLO person detection on frame."""
        return self.yolo.predict(frame, classes=0, verbose=False)

    def run_deepsort(self, frame, yolo_results) -> list[dict]:
        """Update DeepSORT with YOLO detections. Returns confirmed track dicts."""
        frame_h, frame_w = frame.shape[:2]
        detections = []
        for out in yolo_results:
            for box in out.boxes:
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                x1 = max(0, min(x1, frame_w - 1))
                y1 = max(0, min(y1, frame_h - 1))
                x2 = max(0, min(x2, frame_w))
                y2 = max(0, min(y2, frame_h))
                prob = round(box.conf[0].item(), 2)
                if prob < CONF_THRESHOLD:
                    continue
                cropped = frame[y1:y2, x1:x2]
                if cropped.size == 0:
                    continue
                feature = self.extract_reid(cropped)
                detections.append(
                    DeepSORTDetection(
                        tlwh=np.array([x1, y1, x2 - x1, y2 - y1], dtype=np.float64),
                        confidence=prob,
                        feature=feature,
                    )
                )
        self.deepsort.predict()
        self.deepsort.update(detections)
        tracks = []
        for track in self.deepsort.tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue
            bbox = track.to_tlbr()
            tracks.append(
                {
                    "track_id": track.track_id,
                    "x1": max(0, int(bbox[0])),
                    "y1": max(0, int(bbox[1])),
                    "x2": min(frame_w, int(bbox[2])),
                    "y2": min(frame_h, int(bbox[3])),
                }
            )
        return tracks

    def extract_reid(self, crop: np.ndarray):
        """Extract ReID embedding as numpy array from a BGR crop."""
        pil = PILImage.fromarray(crop)
        with torch.no_grad():
            emb = extract_feature_from_img(pil, self.reid)
        return emb.cpu().numpy().flatten()

    def extract_reid_tensor(self, crop: np.ndarray):
        """Extract ReID embedding as tensor (for compare_images functions)."""
        pil = PILImage.fromarray(crop)
        with torch.no_grad():
            return extract_feature_from_img(pil, self.reid)

    def extract_reid_batch(self, crops: list[np.ndarray]):
        """Batch ReID extraction; returns list of tensors."""
        pils = [PILImage.fromarray(c) for c in crops]
        with torch.no_grad():
            return extract_feature_from_img_batch(pils, self.reid)

    def compare(self, emb1, emb2, threshold: float = 0.7) -> bool:
        return compare_images(emb1, emb2, threshold=threshold)

    def compare_batch(self, embedding, embeddings, threshold: float = 0.7) -> bool:
        return compare_images_batch(embedding, embeddings, threshold=threshold)
