"""YOLO detector (v8/v11/v26) with automatic TensorRT export on first run."""

import os
import shutil

from .base import DetectorModel, Detection
from .registry import ModelRegistry, MODELS_PATH


def _load_yolo_trt(model_path: str):
    """Load YOLO with automatic TensorRT export for Orin AGX."""
    from ultralytics import YOLO

    cache_dir = os.environ.get("TENSORRT_CACHE_DIR")
    engine_name = os.path.basename(model_path).replace(".pt", ".engine")
    engine_path = (
        os.path.join(cache_dir, engine_name)
        if cache_dir
        else model_path.replace(".pt", ".engine")
    )

    if cache_dir:
        os.makedirs(cache_dir, exist_ok=True)

    if os.path.exists(engine_path):
        print(f"[TRT] Loading cached engine: {engine_path}")
        return YOLO(engine_path, task="detect")

    model = YOLO(model_path)
    try:
        print(f"[TRT] Exporting {model_path} to TensorRT (first run only)...")
        model.export(format="engine", half=True, device=0, imgsz=640)
        local_engine = model_path.replace(".pt", ".engine")
        if local_engine != engine_path and os.path.exists(local_engine):
            shutil.move(local_engine, engine_path)
        print(f"[TRT] Engine saved: {engine_path}")
        return YOLO(engine_path, task="detect")
    except Exception as e:
        print(f"[TRT] Export failed ({e}), using PyTorch model")
        return model


@ModelRegistry.register("yolo")
class YoloModel(DetectorModel):
    def load(self, config: dict):
        model_path = MODELS_PATH + config["filename"]
        self.model = _load_yolo_trt(model_path)
        self.conf = config.get("conf", 0.6)
        print(f"[YoloModel:{self.name}] loaded from {model_path}")

    def detect(self, image) -> list[Detection]:
        results = self.model.predict(image, verbose=False)
        detections = []
        h, w = image.shape[:2]
        for out in results:
            if out.boxes is None:
                continue
            for box in out.boxes:
                conf = box.conf[0].item()
                if conf < self.conf:
                    continue
                label_id = int(box.cls[0].item())
                label_text = self.translate(self.model.names[label_id])
                det = Detection(label_text, label_id, conf)
                x1, y1, x2, y2 = [round(v) for v in box.xyxy[0].tolist()]
                det.bbox_.x1 = float(x1) / w
                det.bbox_.y1 = float(y1) / h
                det.bbox_.x2 = float(x2) / w
                det.bbox_.y2 = float(y2) / h
                det.bbox_.w = w
                det.bbox_.h = h
                detections.append(det)
        return detections
