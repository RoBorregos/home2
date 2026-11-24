"""YOLO detector (v8/v11/v26) with automatic TensorRT export on first run."""

from .base import DetectorModel, Detection
from .registry import ModelRegistry, MODELS_PATH
from ultralytics import YOLO
from utils.trt_utils import load_yolo_trt


@ModelRegistry.register("yolo")
class YoloModel(DetectorModel):
    def load(self, config: dict):
        model_path = MODELS_PATH + config["filename"]
        use_trt = config.get("use_trt", True)
        if use_trt:
            self.model = load_yolo_trt(model_path)
        else:
            self.model = YOLO(model_path, task="detect")
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
