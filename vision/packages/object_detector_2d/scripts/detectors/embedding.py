"""Few-shot object recognition: class-agnostic YOLOE-pf boxes + frozen
DINOv2-B embeddings matched via gallery_matcher.Gallery — no retraining to add an object; out-of-gallery reports "unknown". See the benchmark README for calibration."""

import numpy as np
from PIL import Image

from embedding_gallery.image_embedder import ImageEmbedder
from embedding_gallery.gallery_matcher import (
    DEFAULT_MAX_BOX_AREA_FRAC,
    UNKNOWN,
    Gallery,
)

from .base import Detection, DetectorModel
from .registry import MODELS_PATH, ModelRegistry


@ModelRegistry.register("embedding")
class EmbeddingModel(DetectorModel):
    def load(self, config: dict):
        self.box_model = ModelRegistry.get(config["box_model"])
        self.embedder = ImageEmbedder(
            config["backbone"], use_trt=config.get("use_trt", True)
        ).load()
        self.gallery = Gallery.load(MODELS_PATH + config["gallery_dir"])
        self.publish_unknown = config.get("publish_unknown", False)
        # Drop oversized boxes before they're even embedded — cheaper than
        # embedding them and letting the matching decision alone.
        self.max_box_area_frac = config.get(
            "max_box_area_frac", DEFAULT_MAX_BOX_AREA_FRAC
        )
        # Stable per-run label -> class_id; gallery objects have no fixed
        # numeric id the way a YOLO .names dict does.
        self._label_ids = {
            name: i for i, name in enumerate(sorted(self.gallery.thresholds))
        }
        print(
            f"[EmbeddingModel:{self.name}] backbone={config['backbone']} "
            f"gallery={len(self.gallery.thresholds)} objects "
            f"box_model={config['box_model']}"
        )

    def detect(self, image) -> list[Detection]:
        # A fresh install has no enrolled objects — every match would be
        # UNKNOWN anyway, so skip the box proposer/embedder entirely.
        if not self.gallery.thresholds and not self.publish_unknown:
            return []

        box_detections = self.box_model.detect(image)
        if not box_detections:
            return []

        h, w = image.shape[:2]
        frame_area = w * h
        crops = []
        boxes_px = []
        for det in box_detections:
            x1 = max(0, int(det.bbox_.x1 * w))
            y1 = max(0, int(det.bbox_.y1 * h))
            x2 = min(w, int(det.bbox_.x2 * w))
            y2 = min(h, int(det.bbox_.y2 * h))
            if x2 <= x1 or y2 <= y1:
                continue
            if (x2 - x1) * (y2 - y1) > self.max_box_area_frac * frame_area:
                continue
            # image is BGR (cv2 convention, same as yolo.py/yolo_e.py); PIL
            # and the DINOv2/CLIP transforms both expect RGB.
            crop_rgb = np.asarray(image[y1:y2, x1:x2])[:, :, ::-1]
            crops.append(Image.fromarray(crop_rgb))
            boxes_px.append((x1, y1, x2, y2))

        if not crops:
            return []

        # One batched forward pass for every crop in the frame, per the plan.
        embeddings = self.embedder.embed_batch(crops)
        matches = self.gallery.match_batch(embeddings)

        detections = []
        for (x1, y1, x2, y2), (label, sim, _margin) in zip(boxes_px, matches):
            if label == UNKNOWN and not self.publish_unknown:
                continue
            class_id = self._label_ids.get(label, -1)
            det = Detection(self.translate(label), class_id, float(sim))
            det.bbox_.x1 = x1 / w
            det.bbox_.y1 = y1 / h
            det.bbox_.x2 = x2 / w
            det.bbox_.y2 = y2 / h
            det.bbox_.w = w
            det.bbox_.h = h
            detections.append(det)
        return detections
