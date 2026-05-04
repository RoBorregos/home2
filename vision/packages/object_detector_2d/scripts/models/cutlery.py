import os

from .base import DetectorModel, Detection
from .registry import ModelRegistry

_SPANISH_TO_ENGLISH = {"cuchara": "spoon", "tenedor": "fork", "cuchillo": "knife"}


@ModelRegistry.register("cutlery")
class CutleryModel(DetectorModel):
    def load(self, config: dict):
        from ament_index_python.packages import get_package_share_directory
        from vision_general.utils.trt_utils import load_yolo_trt

        model_path = os.path.join(
            get_package_share_directory("vision_general"),
            "Utils",
            "models",
            "cutlery.pt",
        )
        self.model = load_yolo_trt(model_path)
        self.conf = config.get("conf", 0.3)
        print(f"[CutleryModel] loaded from {model_path}")

    def detect(self, image) -> list[Detection]:
        results = self.model(image, verbose=False, classes=[0, 1, 3])
        detections = []
        h, w = image.shape[:2]
        for res in results:
            for box in res.boxes:
                conf = box.conf.item()
                if conf < self.conf:
                    continue
                cls_id = int(box.cls.item())
                label = (
                    self.model.names[cls_id]
                    if hasattr(self.model, "names")
                    else str(cls_id)
                )
                label = _SPANISH_TO_ENGLISH.get(label.lower(), label)
                det = Detection(label, cls_id, conf)
                x1, y1, x2, y2 = [round(v) for v in box.xyxy[0].tolist()]
                det.bbox_.x1 = float(x1) / w
                det.bbox_.y1 = float(y1) / h
                det.bbox_.x2 = float(x2) / w
                det.bbox_.y2 = float(y2) / h
                det.bbox_.w = w
                det.bbox_.h = h
                detections.append(det)
        return detections
