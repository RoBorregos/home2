# One person from the shared pose model (ObjectDetect2D); a msg is needed so DetectPose can return a list of people.
int32 x1
int32 y1
int32 x2
int32 y2
float32[] keypoints       # 17 COCO keypoints, normalized (0-1), flattened as x0,y0,x1,y1,...
float32[] keypoints_conf  # 17 confidences
