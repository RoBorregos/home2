#ifndef VISION_CONSTANTS_CPP
#define VISION_CONSTANTS_CPP

#include <string>

typedef const std::string conststr;

// ZED CAMERA TOPICS
conststr CAMERA_TOPIC = "/zed/zed_node/rgb/image_rect_color";
conststr DEPTH_IMAGE_TOPIC = "/zed/zed_node/depth/depth_registered";
conststr CAMERA_INFO_TOPIC = "/zed/zed_node/depth/camera_info";

// ZED CAMERA FRAME
conststr CAMERA_FRAME = "zed_left_camera_optical_frame";

// OBJECT 2D TOPICS
conststr DETECTIONS_TOPIC = "/vision/detections";
conststr DETECTIONS_IMAGE_TOPIC = "/vision/detections_image";
conststr DETECTIONS_POSES_TOPIC = "/vision/detection_poses";
conststr DETECTIONS_3D_TOPIC = "/vision/detections_3d";
conststr DETECTIONS_ACTIVE_TOPIC = "/detections_active";
conststr DEBUG_IMAGE_TOPIC = "/vision/debug_image";

// LOCAL CAMERA TOPICS
conststr LOCAL_CAMERA_TOPIC = "vision/local_rbg_image";

// OBJECT DETECTION HANDLER TOPICS
conststr DETECTION_HANDLER_TOPIC_SV = "/vision/detection_handler";

#endif // VISION_CONSTANTS_CPP