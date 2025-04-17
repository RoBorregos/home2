#! /usr/bin/env python3
from detectors.ObjectDetector import ObjectDectector, Detection, ObjectDectectorParams
import torch
import warnings
import cv2
import copy

warnings.filterwarnings("ignore", category=FutureWarning)


class YoloV5ObjectDetector(ObjectDectector):
    def __init__(self, model_path: str, object_detector_params: ObjectDectectorParams):
        super().__init__(model_path, object_detector_params)
        self.loadYoloV5Model()

    def loadYoloV5Model(self):
        self.model = torch.hub.load(
            "ultralytics/yolov5",
            "custom",
            path="/workspace/vision/packages/object_detector_2d/yolo11classes.pt",
            force_reload=False,
        )

    def _inference(self, frame):
        det_frame = copy.deepcopy(frame)
        det_frame = cv2.cvtColor(det_frame, cv2.COLOR_BGR2RGB)
        detections = self.model(det_frame)

        for *xyxy, confidence, _class, names in (
            detections.pandas().xyxy[0].itertuples(index=False)
        ):
            # Normalized [0-1] ymin, xmin, ymax, xmax
            height = frame.shape[1]
            width = frame.shape[0]
            if confidence < self.object_detector_params_.min_score_thresh:
                continue

            detection_ = Detection(names, _class, confidence)
            detection_.bbox_.h, detection_.bbox_.w = height, width
            detection_.bbox_.y1 = float(xyxy[1] / width)
            detection_.bbox_.x1 = float(xyxy[0] / height)
            detection_.bbox_.y2 = float(xyxy[3] / width)
            detection_.bbox_.x2 = float(xyxy[2] / height)
            self.detections_.append(detection_)

        return self.detections_
