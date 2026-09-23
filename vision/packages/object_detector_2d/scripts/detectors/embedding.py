"""Few-shot object recognition: class-agnostic boxes + a frozen DINOv2 crop
embedding matched against a small per-object gallery (gallery_build.py) — no
retraining to add an object. Anything outside the gallery reports as
"unknown" (dropped by default; see `publish_unknown`).

Chosen config (see vision/benchmarks/embedding_gallery/README.md for the
real-data benchmark this comes from — recall@1 ~82%, unknown-rejection ~84%,
excluding a short, evidenced list of shape-ambiguous classes already covered
by yolo_finetuned):
  - box proposer: YOLOE prompt-free (`yoloe-11l-seg-pf.pt`) at conf 0.10 —
    97.1% recall@IoU0.5 on real RCW2026_v2 crops (see
    vision/benchmarks/embedding_gallery/results/box_recall.json). Reused
    as-is via the existing "yolo_e" type; only its boxes are used, its own
    labels are discarded.
  - backbone: DINOv2 ViT-B/14, frozen.
  - matching: gallery_matcher.Gallery (per-class floor + top1-vs-top2 margin).
"""

import numpy as np
from PIL import Image

from .backbone import EmbeddingBackbone
from .base import Detection, DetectorModel
from .gallery_matcher import UNKNOWN, Gallery
from .registry import MODELS_PATH, ModelRegistry


@ModelRegistry.register("embedding")
class EmbeddingModel(DetectorModel):
    def load(self, config: dict):
        self.box_model = ModelRegistry.get(config["box_model"])
        self.backbone = EmbeddingBackbone(config["backbone"]).load()
        self.gallery = Gallery.load(MODELS_PATH + config["gallery_dir"])
        self.publish_unknown = config.get("publish_unknown", False)
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
        box_detections = self.box_model.detect(image)
        if not box_detections:
            return []

        h, w = image.shape[:2]
        crops = []
        boxes_px = []
        for det in box_detections:
            x1 = max(0, int(det.bbox_.x1 * w))
            y1 = max(0, int(det.bbox_.y1 * h))
            x2 = min(w, int(det.bbox_.x2 * w))
            y2 = min(h, int(det.bbox_.y2 * h))
            if x2 <= x1 or y2 <= y1:
                continue
            # image is BGR (cv2 convention, same as yolo.py/yolo_e.py); PIL
            # and the DINOv2/CLIP transforms both expect RGB.
            crop_rgb = np.asarray(image[y1:y2, x1:x2])[:, :, ::-1]
            crops.append(Image.fromarray(crop_rgb))
            boxes_px.append((x1, y1, x2, y2))

        if not crops:
            return []

        # One batched forward pass for every crop in the frame, per the plan.
        embeddings = self.backbone.embed_batch(crops)
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
