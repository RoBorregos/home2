#!/usr/bin/env python3
"""
Wrappers for ROS tracker nodes to enable offline benchmarking.
These wrappers extract the core tracking logic from ROS nodes for use in MOT16 evaluation.
"""

from __future__ import annotations

import sys
import os
from pathlib import Path
from typing import List, Tuple
import numpy as np
import cv2

# Add paths to import tracker modules
vision_scripts = Path(__file__).parent.parent / "scripts"
sys.path.insert(0, str(vision_scripts))

from ultralytics import YOLO

try:
    from norfair import Detection, Tracker as NorfairTracker
    NORFAIR_AVAILABLE = True
except ImportError:
    NORFAIR_AVAILABLE = False

# DeepSORT imports for Hector Tracker
try:
    # Add vision_general package to path (Utils/benchmark -> packages)
    vision_general_base = Path(__file__).parent.parent.parent
    if str(vision_general_base) not in sys.path:
        sys.path.insert(0, str(vision_general_base))

    from vision_general.utils.deep_sort import nn_matching
    from vision_general.utils.deep_sort.detection import Detection as DeepSortDetection
    from vision_general.utils.deep_sort.tracker import Tracker as DeepSORTTracker
    from vision_general.utils.reid_model import load_network, extract_feature_from_img, get_structure
    import torch
    import torch.nn as nn
    from PIL import Image as PILImage
    DEEPSORT_AVAILABLE = True
except ImportError as e:
    DEEPSORT_AVAILABLE = False
    print(f"Warning: DeepSORT not available: {e}")


class BaseOfflineTrackerWrapper:
    """Base class for offline tracker wrappers"""

    def __init__(self, conf_threshold: float = 0.6):
        self.conf_threshold = conf_threshold
        self.frame_count = 0

    def reset(self):
        """Reset tracker state for new sequence"""
        self.frame_count = 0

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        """
        Process a single frame and return detections.
        Returns: List of (track_id, [x, y, w, h])
        """
        raise NotImplementedError


class NewTrackerWrapper(BaseOfflineTrackerWrapper):
    """
    Wrapper for new_tracker.py (BoTSORT with ReID)
    Uses Ultralytics YOLO with BoTSORT tracker
    """

    def __init__(self, conf_threshold: float = 0.6):
        super().__init__(conf_threshold)
        self.model = YOLO("yolov8n.pt")
        self.tracker_config = "botsort.yaml"
        print("Loaded NewTracker (BoTSORT with ReID)")

    def reset(self):
        super().reset()
        self.model.predictor = None

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        self.frame_count += 1

        results = self.model.track(
            frame,
            conf=self.conf_threshold,
            classes=[0],  # person class
            persist=True,
            tracker=self.tracker_config,
            verbose=False,
        )

        detections = []
        for result in results:
            if result.boxes is None or result.boxes.id is None:
                continue

            boxes = result.boxes.xyxy.cpu().numpy()
            ids = result.boxes.id.cpu().numpy().astype(int)

            for box, track_id in zip(boxes, ids):
                x1, y1, x2, y2 = box
                w = x2 - x1
                h = y2 - y1
                detections.append((int(track_id), [float(x1), float(y1), float(w), float(h)]))

        return detections


class OldTrackerWrapper(BaseOfflineTrackerWrapper):
    """
    Wrapper for old_tracker.py
    Uses standard Ultralytics tracking (likely ByteTrack or similar)
    """

    def __init__(self, conf_threshold: float = 0.6):
        super().__init__(conf_threshold)
        self.model = YOLO("yolov8n.pt")
        self.tracker_config = "bytetrack.yaml"
        print("Loaded OldTracker (ByteTrack)")

    def reset(self):
        super().reset()
        self.model.predictor = None

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        self.frame_count += 1

        results = self.model.track(
            frame,
            conf=self.conf_threshold,
            classes=[0],  # person class
            persist=True,
            tracker=self.tracker_config,
            verbose=False,
        )

        detections = []
        for result in results:
            if result.boxes is None or result.boxes.id is None:
                continue

            boxes = result.boxes.xyxy.cpu().numpy()
            ids = result.boxes.id.cpu().numpy().astype(int)

            for box, track_id in zip(boxes, ids):
                x1, y1, x2, y2 = box
                w = x2 - x1
                h = y2 - y1
                detections.append((int(track_id), [float(x1), float(y1), float(w), float(h)]))

        return detections


class TrackerNodeWrapper(BaseOfflineTrackerWrapper):
    """
    Wrapper for tracker_node.py
    Standard tracker node implementation
    """

    def __init__(self, conf_threshold: float = 0.6):
        super().__init__(conf_threshold)
        self.model = YOLO("yolov8n.pt")
        self.tracker_config = "botsort.yaml"
        print("Loaded TrackerNode")

    def reset(self):
        super().reset()
        self.model.predictor = None

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        self.frame_count += 1

        results = self.model.track(
            frame,
            conf=self.conf_threshold,
            classes=[0],
            persist=True,
            tracker=self.tracker_config,
            verbose=False,
        )

        detections = []
        for result in results:
            if result.boxes is None or result.boxes.id is None:
                continue

            boxes = result.boxes.xyxy.cpu().numpy()
            ids = result.boxes.id.cpu().numpy().astype(int)

            for box, track_id in zip(boxes, ids):
                x1, y1, x2, y2 = box
                w = x2 - x1
                h = y2 - y1
                detections.append((int(track_id), [float(x1), float(y1), float(w), float(h)]))

        return detections


class TrackerNodeFregosoWrapper(BaseOfflineTrackerWrapper):
    """
    Wrapper for tracker_node_fregoso.py
    Fregoso's tracker implementation
    """

    def __init__(self, conf_threshold: float = 0.8):  # Fregoso uses 0.8
        super().__init__(conf_threshold)
        self.model = YOLO("yolov8n.pt")
        self.tracker_config = "botsort.yaml"
        print("Loaded TrackerNodeFregoso")

    def reset(self):
        super().reset()
        self.model.predictor = None

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        self.frame_count += 1

        results = self.model.track(
            frame,
            conf=self.conf_threshold,
            classes=[0],
            persist=True,
            tracker=self.tracker_config,
            verbose=False,
        )

        detections = []
        for result in results:
            if result.boxes is None or result.boxes.id is None:
                continue

            boxes = result.boxes.xyxy.cpu().numpy()
            ids = result.boxes.id.cpu().numpy().astype(int)

            for box, track_id in zip(boxes, ids):
                x1, y1, x2, y2 = box
                w = x2 - x1
                h = y2 - y1
                detections.append((int(track_id), [float(x1), float(y1), float(w), float(h)]))

        return detections


class HectorTracker(BaseOfflineTrackerWrapper):
    """
    Wrapper for reid_node.py (DeepSORT with ReID)
    Hector's DeepSORT tracker with person re-identification
    """

    def __init__(self, conf_threshold: float = 0.4,
                 max_cosine_distance: float = 0.3,
                 nn_budget: int = 100):
        super().__init__(conf_threshold)

        if not DEEPSORT_AVAILABLE:
            raise ImportError("DeepSORT is not available. Check vision_general.utils.deep_sort imports.")

        self.detector = YOLO("yolov8n.pt")
        self.max_cosine_distance = max_cosine_distance
        self.nn_budget = nn_budget

        # Initialize DeepSORT
        metric = nn_matching.NearestNeighborDistanceMetric(
            "cosine",
            max_cosine_distance,
            nn_budget
        )
        self.tracker = DeepSORTTracker(metric)

        # Load ReID model
        try:
            structure = get_structure()
            self.reid_model = load_network(structure)
            # Remove classifier head
            if hasattr(self.reid_model, 'classifier') and hasattr(self.reid_model.classifier, 'classifier'):
                self.reid_model.classifier.classifier = nn.Sequential()

            self.use_gpu = torch.cuda.is_available()
            if self.use_gpu:
                self.reid_model = self.reid_model.cuda()

            self.reid_model.eval()
            print(f"Loaded Hector Tracker (DeepSORT + ReID, GPU={self.use_gpu})")
        except Exception as e:
            print(f"Warning: Could not load ReID model: {e}")
            self.reid_model = None

    def reset(self):
        super().reset()
        # Reset DeepSORT tracker
        metric = nn_matching.NearestNeighborDistanceMetric(
            "cosine",
            self.max_cosine_distance,
            self.nn_budget
        )
        self.tracker = DeepSORTTracker(metric)

    def _extract_reid_feature(self, image_bgr: np.ndarray, bbox: Tuple[int, int, int, int]) -> np.ndarray:
        """Extract ReID feature from person crop"""
        if self.reid_model is None:
            return None

        try:
            x1, y1, x2, y2 = bbox
            crop = image_bgr[y1:y2, x1:x2]

            if crop.size == 0:
                return None

            # Convert to RGB PIL Image
            crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
            pil_img = PILImage.fromarray(crop_rgb)

            # Extract feature
            with torch.no_grad():
                feature = extract_feature_from_img(pil_img, self.reid_model)
                if feature is None:
                    return None

                feature = feature.squeeze()
                if isinstance(feature, torch.Tensor):
                    feature = feature.detach()
                    if feature.is_cuda:
                        feature = feature.cpu()
                    feature = feature.numpy()
                else:
                    feature = np.array(feature)

                # L2 normalize
                feature = feature.astype(np.float32)
                norm = np.linalg.norm(feature) + 1e-9
                feature = feature / norm

                return feature
        except Exception as e:
            return None

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        self.frame_count += 1

        # YOLO detection
        results = self.detector.predict(
            frame,
            conf=self.conf_threshold,
            classes=[0],  # person class
            verbose=False,
        )

        # Convert to DeepSORT format
        detections = []
        bboxes_xyxy = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())

                if conf < self.conf_threshold:
                    continue

                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                w = x2 - x1
                h = y2 - y1

                # Extract ReID feature
                feature = self._extract_reid_feature(frame, (x1, y1, x2, y2))

                # Create DeepSORT detection
                # DeepSORT expects [x, y, w, h] format
                bbox_xywh = np.array([x1, y1, w, h], dtype=np.float32)

                detection = DeepSortDetection(bbox_xywh, conf, feature)
                detections.append(detection)
                bboxes_xyxy.append((x1, y1, x2, y2))

        # Update DeepSORT tracker
        self.tracker.predict()
        self.tracker.update(detections)

        # Get tracked objects
        tracked_detections = []
        for track in self.tracker.tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue

            # Get bbox in [x, y, w, h] format
            bbox = track.to_tlwh()
            x, y, w, h = bbox

            tracked_detections.append((int(track.track_id), [float(x), float(y), float(w), float(h)]))

        return tracked_detections


class NorfairNodeWrapper(BaseOfflineTrackerWrapper):
    """
    Wrapper for norfair_tracker.py
    Norfair 2-point tracker with YOLO detection
    """

    def __init__(self, conf_threshold: float = 0.6,
                 distance_threshold: float = 65.0,
                 hit_counter_max: int = 25,
                 init_delay: int = 2):
        super().__init__(conf_threshold)

        if not NORFAIR_AVAILABLE:
            raise ImportError("Norfair is not installed")

        self.detector = YOLO("yolov8n.pt")
        self.distance_threshold = distance_threshold
        self.hit_counter_max = hit_counter_max
        self.init_delay = init_delay

        self.tracker = NorfairTracker(
            distance_function=self._two_point_distance,
            distance_threshold=distance_threshold,
            hit_counter_max=hit_counter_max,
            initialization_delay=init_delay,
        )
        print("Loaded NorfairNode (2-point tracker)")

    def reset(self):
        super().reset()
        self.tracker = NorfairTracker(
            distance_function=self._two_point_distance,
            distance_threshold=self.distance_threshold,
            hit_counter_max=self.hit_counter_max,
            initialization_delay=self.init_delay,
        )

    def _two_point_distance(self, detection: Detection, tracked_obj) -> float:
        """Average Euclidean distance over 2 points"""
        det_pts = detection.points
        trk_pts = tracked_obj.estimate

        if det_pts is None or trk_pts is None:
            return 1e9
        if det_pts.shape != trk_pts.shape:
            det_pts = det_pts[:1]
            trk_pts = trk_pts[:1]

        d = np.linalg.norm(det_pts - trk_pts, axis=1)
        return float(np.mean(d))

    def _match_track_to_bbox(self, track_obj, bboxes: List) -> int:
        """Match track to closest bbox by center point"""
        if not bboxes:
            return -1

        ex, ey = track_obj.estimate[0]  # center point
        best_idx = -1
        best_d = 1e18

        for idx, bbox in enumerate(bboxes):
            x1, y1, x2, y2 = bbox
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            d = (cx - ex) ** 2 + (cy - ey) ** 2
            if d < best_d:
                best_d = d
                best_idx = idx

        return best_idx

    def process_frame(self, frame: np.ndarray) -> List[Tuple[int, List[float]]]:
        self.frame_count += 1

        # YOLO detection
        results = self.detector.predict(
            frame,
            conf=self.conf_threshold,
            classes=[0],
            verbose=False,
        )

        # Convert to Norfair detections and store bboxes
        norfair_detections = []
        bboxes = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())

                if conf < self.conf_threshold:
                    continue

                # 2-point representation
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                tcx = cx
                tcy = float(y1)

                points = np.array([[cx, cy], [tcx, tcy]], dtype=np.float32)
                norfair_detections.append(
                    Detection(
                        points=points,
                        scores=np.array([conf, conf], dtype=np.float32),
                    )
                )
                bboxes.append((x1, y1, x2, y2))

        # Update tracker
        tracks = self.tracker.update(norfair_detections)

        # Match tracks to bboxes
        detections = []
        for track in tracks:
            bbox_idx = self._match_track_to_bbox(track, bboxes)
            if bbox_idx >= 0:
                x1, y1, x2, y2 = bboxes[bbox_idx]
                w = x2 - x1
                h = y2 - y1
                detections.append((int(track.id), [float(x1), float(y1), float(w), float(h)]))

        return detections


# Factory function to create tracker wrappers
def create_tracker(tracker_name: str, **kwargs) -> BaseOfflineTrackerWrapper:
    """
    Factory function to create tracker wrappers.

    Args:
        tracker_name: Name of tracker ('new_tracker', 'old_tracker', 'tracker_node',
                      'tracker_node_fregoso', 'norfair_node', 'hector_tracker')
        **kwargs: Additional arguments for tracker initialization

    Returns:
        Tracker wrapper instance
    """
    trackers = {
        'new_tracker': NewTrackerWrapper,
        'old_tracker': OldTrackerWrapper,
        'tracker_node': TrackerNodeWrapper,
        'tracker_node_fregoso': TrackerNodeFregosoWrapper,
        'norfair_node': NorfairNodeWrapper,
        'hector_tracker': HectorTracker,
    }

    if tracker_name not in trackers:
        raise ValueError(f"Unknown tracker: {tracker_name}. Available: {list(trackers.keys())}")

    return trackers[tracker_name](**kwargs)
