from ObjectDetector import ObjectDectector, Detection, BBOX, ObjectDectectorParams
import torch


class YoloV5ObjectDetector(ObjectDectector):
    def __init__(self, model_path : str, object_detector_params : ObjectDectectorParams):
        super.__init__(model_path, object_detector_params)

    def loadYoloV5Model(self):
        self.model = torch.hub.load(
            "ultralytics/yolov5",
            "custom",
            path=self.model_path_,
            force_reload=False,
        )

    def _inference(self, frame):
        detections = self.model(frame)

        for *xyxy, confidence, _class, names in detections.pandas().xyxy[0].itertuples(index=False):
            # Normalized [0-1] ymin, xmin, ymax, xmax
            height = frame.shape[1]
            width = frame.shape[0]
            if confidence < self.object_detector_params_.min_score_thresh:
                continue
            
            detection_ = Detection(names, _class, confidence)
            detection_.bbox_.h = height, detection_.bbox_.w = width
            detection_.bbox_.y1 = xyxy[1] / width 
            detection_.bbox_.x1 = xyxy[0] / height 
            detection_.bbox_.y2 = xyxy[3] / width 
            detection_.bbox_.x2 = xyxy[2] / height 
            self.detections_.append(detection_)   

        return self.detections_  




        