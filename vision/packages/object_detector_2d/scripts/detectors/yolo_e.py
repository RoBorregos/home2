"""YOLOE zero-shot segmentation detector with dynamic class setting."""

import os
import sys
import warnings

from .base import DetectorModel, Detection
from .registry import ModelRegistry, MODELS_PATH

warnings.filterwarnings("ignore", category=FutureWarning)


class _SuppressStderr:
    """Suppress C-level stderr (YOLOE spams on load/predict)."""

    def __enter__(self):
        self._fd = sys.stderr.fileno()
        self._dup = os.dup(self._fd)
        self._devnull = open(os.devnull, "w")
        os.dup2(self._devnull.fileno(), self._fd)

    def __exit__(self, *_):
        os.close(self._fd)
        os.dup2(self._dup, self._fd)
        os.close(self._dup)
        self._devnull.close()


@ModelRegistry.register("yolo_e")
class YoloEModel(DetectorModel):
    def load(self, config: dict):
        from ultralytics import YOLOE

        model_path = MODELS_PATH + config["filename"]
        with _SuppressStderr():
            self.model = YOLOE(model_path)
        self.conf = config.get("conf", 0.5)
        print(f"[YoloEModel:{self.name}] loaded from {model_path}")

    def set_classes(self, classes: list):
        """Zero-shot class update (call before detect())."""
        self.model.set_classes(classes, self.model.get_text_pe(classes))

    def detect(self, image) -> list[Detection]:
        with _SuppressStderr():
            results = self.model.predict(image, verbose=False)
        detections = []
        h, w = image.shape[:2]
        for out in results:
            if out.boxes is None or out.masks is None:
                continue
            for box, mask in zip(out.boxes, out.masks):
                conf = box.conf[0].item()
                if conf < self.conf:
                    continue
                label_id = int(box.cls[0].item())
                label_text = self.model.names[label_id]
                det = Detection(label_text, label_id, conf)
                x1, y1, x2, y2 = [round(v) for v in box.xyxy[0].tolist()]
                det.bbox_.x1 = float(x1) / w
                det.bbox_.y1 = float(y1) / h
                det.bbox_.x2 = float(x2) / w
                det.bbox_.y2 = float(y2) / h
                det.bbox_.w = w
                det.bbox_.h = h
                det.mask = mask.xy[0].tolist()
                detections.append(det)
        return detections
