# One person detected by the shared pose model (pose_node).
# Bounding box in pixels, relative to the region that was analysed.
int32 x1
int32 y1
int32 x2
int32 y2
float32 confidence

# 17 COCO keypoints, flattened as x0,y0,x1,y1,...
float32[] keypoints_xy    # pixels, relative to the region
float32[] keypoints_xyn   # normalized (0-1) to the region
float32[] keypoints_conf  # 17 confidences
