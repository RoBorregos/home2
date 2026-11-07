"""YOLO pose model (yolo11m-pose) for the DetectPose service."""

from ultralytics import YOLO

from .base import Detection, DetectorModel
from .registry import MODELS_PATH, ModelRegistry
from .yolo import _load_yolo_trt

PERSON_CLASS_ID = 0


@ModelRegistry.register("yolo_pose")
class YoloPoseModel(DetectorModel):
    def load(self, config: dict):
        model_path = MODELS_PATH + config["filename"]
        if config.get("use_trt", True):
            self.model = _load_yolo_trt(model_path, task="pose")
        else:
            self.model = YOLO(model_path, task="pose")
        self.conf = config.get("conf", 0.25)
        print(f"[YoloPoseModel:{self.name}] loaded from {model_path}")

    def estimate(self, image, conf: float | None = None):
        """Raw ultralytics result (boxes + keypoints) for one image, or None."""
        results = self.model(
            image, verbose=False, conf=conf if conf and conf > 0 else self.conf
        )
        return results[0] if results else None

    def detect(self, image) -> list[Detection]:
        """Person boxes only, so the model also fits the DetectorModel API."""
        result = self.estimate(image)
        if result is None or result.boxes is None:
            return []
        h, w = image.shape[:2]
        detections = []
        for box in result.boxes:
            det = Detection("person", PERSON_CLASS_ID, box.conf[0].item())
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            det.bbox_.x1, det.bbox_.y1 = x1 / w, y1 / h
            det.bbox_.x2, det.bbox_.y2 = x2 / w, y2 / h
            det.bbox_.w, det.bbox_.h = w, h
            detections.append(det)
        return detections
