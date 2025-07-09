#! /usr/bin/env python3
from detectors.ObjectDetector import ObjectDectector, Detection, ObjectDectectorParams
from ultralytics import YOLO
import warnings


warnings.filterwarnings("ignore", category=FutureWarning)


class YoloV8ObjectDetector(ObjectDectector):
    def __init__(self, model_path: str, object_detector_params: ObjectDectectorParams):
        super().__init__(model_path, object_detector_params)
        self.loadModel()

    def loadModel(self):
        self.model = YOLO(self.model_path_)
        print("Model loaded: ", self.model_path_)

    def _inference(self, frame):
        # Execute prediction for specified categories on an image
        results = self.model.predict(frame, verbose=False)

        return self._generate_detections(results, frame)

    def _generate_detections(self, outs, frame):
        for out in outs:
            if out.boxes is None:
                continue
            for box in out.boxes:
                confidence = box.conf[0].item()
                if confidence < 0.6:
                    continue
                label = int(box.cls[0].item())
                label_id = self.model.names[label]
                detection_ = Detection(label_id, label, confidence)
                x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                width, height = frame.shape[1], frame.shape[0]
                detection_.bbox_.x1 = float(x1) / width
                detection_.bbox_.y1 = float(y1) / height
                detection_.bbox_.x2 = float(x2) / width
                detection_.bbox_.y2 = float(y2) / height
                detection_.bbox_.w = width
                detection_.bbox_.h = height

                self.detections_.append(detection_)

        return self.detections_
