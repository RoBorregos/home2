#! /usr/bin/env python3
from detectors.ObjectDetector import Detection, ObjectDectectorParams
from detectors.ZeroShotObjectDetector import ZeroShotObjectDetector
from ultralytics import YOLOE
import warnings

import sys
import os


class SuppressStream(object):
    def __init__(self):
        self.orig_stream_fileno = sys.stderr.fileno()

    def __enter__(self):
        self.orig_stream_dup = os.dup(self.orig_stream_fileno)
        self.devnull = open(os.devnull, "w")
        os.dup2(self.devnull.fileno(), self.orig_stream_fileno)

    def __exit__(self, type, value, traceback):
        os.close(self.orig_stream_fileno)
        os.dup2(self.orig_stream_dup, self.orig_stream_fileno)
        os.close(self.orig_stream_dup)
        self.devnull.close()


warnings.filterwarnings("ignore", category=FutureWarning)


class YoloEObjectDetector(ZeroShotObjectDetector):
    def __init__(self, model_path: str, object_detector_params: ObjectDectectorParams):
        super().__init__(model_path, object_detector_params)
        self.loadYoloEModel()

    def loadYoloEModel(self):
        with SuppressStream():
            self.model = YOLOE(self.model_path_)
        print("Model loaded: ", self.model_path_)

    def _inference(self, frame):
        # Execute prediction for specified categories on an image
        with SuppressStream():
            results = self.model.predict(frame, verbose=False)

        return self._generate_detections(results, frame)

    def _generate_detections(self, outs, frame):
        for out in outs:
            if out.boxes is None or out.masks is None:
                continue
            for box, mask in zip(out.boxes, out.masks):
                confidence = box.conf[0].item()
                if confidence < self.object_detector_params_.min_score_thresh:
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
                detection_.mask = mask.xy[0].tolist()

                self.detections_.append(detection_)

        return self.detections_

    def set_classes(self, classes):
        self.model.set_classes(classes, self.model.get_text_pe(classes))
        print("Classes set: ", classes)
