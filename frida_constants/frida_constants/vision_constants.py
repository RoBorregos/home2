# ZED CAMERA TOPICS
CAMERA_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEPTH_IMAGE_TOPIC = "/zed/zed_node/depth/depth_registered"
CAMERA_INFO_TOPIC = "/zed/zed_node/depth/camera_info"

# Activate this if you want to used the image of the ZED in the simulation
# CAMERA_TOPIC = "/zed/image_raw"
# DEPTH_IMAGE_TOPIC = "/zed/depth/image_raw"
# CAMERA_INFO_TOPIC = "/zed/depth/camera_info"

# ZED CAMERA FRAME
CAMERA_FRAME = "zed_left_camera_optical_frame"

# OBJECT 2D TOPICS
DETECTIONS_TOPIC = "/vision/detections"
DETECTIONS_IMAGE_TOPIC = "/vision/detections_image"
DETECTIONS_POSES_TOPIC = "/vision/detection_poses"
DETECTIONS_3D_TOPIC = "/vision/detections_3d"
DETECTIONS_ACTIVE_TOPIC = "/detections_active"
DEBUG_IMAGE_TOPIC = "/vision/debug_image"

# ZERO SHOT OBJECT DETECTOR TOPICS
ZERO_SHOT_DETECTIONS_TOPIC = "/vision/zero_shot_detections"
ZERO_SHOT_DETECTIONS_IMAGE_TOPIC = "/vision/zero_shot_detections_image"
ZERO_SHOT_DETECTIONS_POSES_TOPIC = "/vision/zero_shot_detections_poses"
ZERO_SHOT_DETECTIONS_3D_TOPIC = "/vision/zero_shot_detections_3d"
ZERO_SHOT_DETECTIONS_ACTIVE_TOPIC = "/vision/zero_shot_detections_active"
SET_DETECTOR_CLASSES_SERVICE = "/vision/set_detector_classes"

ZERO_SHOT_DEFAULT_CLASSES = [
    "yellow_bowl",
    "squash",
    "apple",
    "coke_bottle",
    "fanta_can",
    "orange",
    "cup",
]

ZERO_SHOT_MODEL = "yoloe-11l-seg.pt"

# LOCAL CAMERA TOPICS
LOCAL_CAMERA_TOPIC = "/vision/local_rbg_image"

# OBJECT DETECTION HANDLER TOPICS
DETECTION_HANDLER_TOPIC_SRV = "/vision/detection_handler"

# Tracker node
SET_TARGET_TOPIC = "/vision/set_tracking_target"
SET_TARGET_BY_TOPIC = "/vision/set_tracking_target_by"
RESULTS_TOPIC = "/vision/tracking_results"
TRACKER_IMAGE_TOPIC = "/vision/tracker_image"
CENTROID_TOIC = "/vision/tracker_centroid"

# Pointing
POINTING_OBJECT_SERVICE = "/vision/pointing_object_service"
POINTING_DETECTION_IMAGE_TOPIC = "/vision/pointing_detection_image"
SET_POINTING_OBJECT_CLASSES_SERVICE = "/vision/set_pointing_object_classes"

# Moondream
BEVERAGE_TOPIC = "/vision/beverage_location"
# PERSON_DESCRIPTION_TOPIC = "/vision/person_description"
PERSON_POSTURE_TOPIC = "/vision/person_posture"
QUERY_TOPIC = "/vision/query"
PERSON_POSTURE_TOPIC = "/vision/person_posture"
CROP_QUERY_TOPIC = "/vision/crop_query"

# Face recognition
SAVE_NAME_TOPIC = "/vision/new_name"
FOLLOW_TOPIC = "/vision/follow_face"
PERSON_LIST_TOPIC = "/vision/person_list"
PERSON_NAME_TOPIC = "/vision/person_detected_name"
FACE_RECOGNITION_IMAGE = "/vision/face_recognition_image"
FOLLOW_BY_TOPIC = "/vision/follow_by_name"

# Receptionist commands node
CHECK_PERSON_TOPIC = "/vision/receptionist/detect_person"
FIND_SEAT_TOPIC = "/vision/receptionist/find_seat"
IMAGE_TOPIC_RECEPTIONIST = "/vision/receptionist/img_person_detecion"

# GPSR commands node
COUNT_BY_COLOR_TOPIC = "/vision/gpsr/count_by_color"
COUNT_BY_CLOTHES_TOPIC = "/vision/gpsr/count_by_clothes"
COUNT_BY_PERSON_TOPIC = "/vision/gpsr/count_by_person"
COUNT_BY_OBJECTS_TOPIC = "/vision/gpsr/count_by_objects"
COUNT_BY_POSE_TOPIC = "/vision/gpsr/count_by_pose"
COUNT_BY_GESTURE_TOPIC = "/vision/gpsr/count_by_gesture"
IMAGE_TOPIC = "/vision/gpsr/img_detection"
POSE_GESTURE_TOPIC = "/vision/gpsr/pose_gesture_detection"

# Storing Groceries Commands node
SHELF_DETECTION_TOPIC = "/vision/storing_grocPeries/shelf_detection"
