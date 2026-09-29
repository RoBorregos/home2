"""Few-shot object recognition: class-agnostic boxes + a frozen DINOv2 crop
embedding matched against a small per-object gallery (gallery_build.py) — no
retraining to add an object. Anything outside the gallery reports as
"unknown" (dropped by default; see `publish_unknown`).

Box proposer: YOLOE prompt-free (`yoloe-11l-seg-pf.pt`, conf 0.10, the
"yolo_e" type) — only its boxes are used, its own labels are discarded.
Backbone: DINOv2 ViT-B/14, frozen. Matching: gallery_matcher.Gallery
(per-class floor + top1-vs-top2 margin). See
vision/benchmarks/embedding_gallery/README.md for the benchmark this was
calibrated against.

TensorRT (config "use_trt": True, default): backbone runs via onnxruntime's
TensorrtExecutionProvider instead of plain PyTorch (932ms -> 209ms for an
8-crop batch on a Jetson Orin, negligible accuracy loss). First load after a
cache miss builds the engine (a few minutes); reused after that.
"""

import numpy as np
from PIL import Image

from .backbone import EmbeddingBackbone
from .base import Detection, DetectorModel
from .gallery_matcher import UNKNOWN, Gallery
from .registry import MODELS_PATH, ModelRegistry

# The class-agnostic box proposer sometimes returns a box covering most of
# the frame (background/desk clutter, not a single object). A wide crop
# like that can score a deceptively high similarity against a small
# gallery — with few gallery objects, the per-class margin check has
# little to discriminate against (see gallery_matcher.py's match_batch
# docstring), so nothing else rejects it. Override per-model via
# MODEL_CONFIGS[...]["max_box_area_frac"] in registry.py if a legitimate
# object genuinely needs to fill more of the frame than this.
DEFAULT_MAX_BOX_AREA_FRAC = 0.5


@ModelRegistry.register("embedding")
class EmbeddingModel(DetectorModel):
    def load(self, config: dict):
        self.box_model = ModelRegistry.get(config["box_model"])
        self.backbone = EmbeddingBackbone(
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
