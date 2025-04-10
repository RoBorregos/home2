# ZED CAMERA TOPICS
CAMERA_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEPTH_IMAGE_TOPIC = "/zed/zed_node/depth/depth_registered"
CAMERA_INFO_TOPIC = "/zed/zed_node/depth/camera_info"

# ZED CAMERA FRAME
CAMERA_FRAME = "zed_left_camera_optical_frame"

# OBJECT 2D TOPICS
DETECTIONS_TOPIC = "/vision/detections"
DETECTIONS_IMAGE_TOPIC = "/vision/detections_image"
DETECTIONS_POSES_TOPIC = "/vision/detection_poses"
DETECTIONS_3D_TOPIC = "/vision/detections_3d"
DETECTIONS_ACTIVE_TOPIC = "/detections_active"
DEBUG_IMAGE_TOPIC = "/vision/debug_image"

# LOCAL CAMERA TOPICS
LOCAL_CAMERA_TOPIC = "/vision/local_rbg_image"

# OBJECT DETECTION HANDLER TOPICS
DETECTION_HANDLER_TOPIC_SV = "/vision/detection_handler"

# Tracker node
SET_TARGET_TOPIC = "/vision/set_tracking_target"
SET_TARGET_BY_TOPIC = "/vision/set_tracking_target_by"
RESULTS_TOPIC = "/vision/tracking_results"
TRACKER_IMAGE_TOPIC = "/vision/tracker_image"

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
VISION_FRAME_TOPIC = "/vision/person_frame"
FOLLOW_BY_TOPIC = "/vision/follow_by_name"

# Receptionist commands node
CHECK_PERSON_TOPIC = "/vision/receptionist/detect_person"
FIND_SEAT_TOPIC = "/vision/receptionist/find_seat"
IMAGE_TOPIC = "/vision/receptionist/img_person_detecion"

# GPSR commands node
COUNT_BY_COLOR_TOPIC = "/vision/gpsr/count_by_color"
COUNT_BY_CLOTHES_TOPIC = "/vision/gpsr/count_by_clothes"
COUNT_BY_PERSON_TOPIC = "/vision/gpsr/count_by_person"
COUNT_BY_OBJECTS_TOPIC = "/vision/gpsr/count_by_objects"
COUNT_BY_GESTURES_TOPIC = "/vision/gpsr/count_by_gestures"
COUNT_BY_POSE_TOPIC = "/vision/gpsr/count_by_pose"
IMAGE_TOPIC = "/vision/gpsr/img_detection"
