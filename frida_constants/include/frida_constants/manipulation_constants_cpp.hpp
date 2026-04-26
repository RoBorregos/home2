#ifndef MANIPULATION_CONSTANTS_CPP
#define MANIPULATION_CONSTANTS_CPP

#include <string>

typedef const std::string conststr;

// Existing constants
conststr ZED_POINT_CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered";
conststr POINT_CLOUD_TOPIC = "/point_cloud";
conststr REMOVE_PC_TEST = "/manipulation/test_service";
conststr REMOVE_PLANE_SERVICE = "/manipulation/extract_plane";
conststr CLUSTER_OBJECT_SERVICE =
    "/manipulation/cluster_object"; // Kept original
conststr REMOVE_VERTICAL_PLANE_SERVICE = "/manipulation/remove_vertical_plane";
conststr ADD_PICK_PRIMITIVES_SERVICE = "/manipulation/add_pick_primitives";
conststr ADD_COLLISION_SERVICE = "/manipulation/add_collision_objects";
conststr PICK_PERCEPTION_SERVICE =
    "/manipulation/pick_perception_service"; // Kept original
conststr PLACE_PERCEPTION_SERVICE =
    "/manipulation/place_perception_service"; // Kept original
conststr PLACE_CLOUD_TOPIC_PUBLISHER = "/manipulation/place_cloud_publisher";
conststr GET_PLANE_BBOX_SERVICE = "/manipulation/get_plane_bbox";
conststr ZED_CAMERA_FRAME = "zed_left_camera_frame";

// New constants
conststr EEF_LINK_NAME = "link_eef";
conststr EEF_CONTACT_LINKS = "link_eef,link_6";

const double DEG2RAD = 3.141592653589793 / 180.0;
const double RAD2DEG = 180.0 / 3.141592653589793;

// xArm Constants
conststr XARM_SETMODE_SERVICE = "/xarm/set_mode";
conststr XARM_SETSTATE_SERVICE = "/xarm/set_state";
const int MOVEIT_MODE = 1;
const int JOINT_VELOCITY_MODE = 4;

conststr XARM_MOVEVELOCITY_SERVICE = "/xarm/vc_set_joint_velocity";
conststr SET_JOINT_VELOCITY_SERVICE = "/manipulation/set_joint_velocity";

const bool ALWAYS_SET_MODE = true;

conststr MOVE_JOINTS_ACTION_SERVER = "/manipulation/move_joints_action_server";
conststr MOVE_TO_POSE_ACTION_SERVER =
    "/manipulation/move_to_pose_action_server";

const double PICK_VELOCITY = 0.15;
const double PICK_ACCELERATION = 0.15;
conststr PICK_PLANNER = "RRTConnect";

conststr PICK_MOTION_ACTION_SERVER_NODE = "manipulation/pick_motion_server";
conststr PICK_MOTION_ACTION_SERVER = "/manipulation/pick_motion_action_server";

conststr GRASP_DETECTION_SERVICE = "/manipulation/detect_grasps";

conststr PICK_ACTION_SERVER = "/manipulation/pick_action_server";

conststr ATTACH_COLLISION_OBJECT_SERVICE =
    "/manipulation/attach_collision_object";
conststr GET_COLLISION_OBJECTS_SERVICE = "/manipulation/get_collision_objects";
conststr PICK_OBJECT_NAMESPACE = "frida_pick_object_";

conststr GRASP_POINTCLOUD_TOPIC = "/manipulation/grasp_pcl";
conststr GRASP_MARKER_TOPIC = "/manipulation/grasp_markers";

// Additional new constants
conststr REMOVE_COLLISION_OBJECT_SERVICE =
    "/manipulation/remove_collision_object";
conststr DEBUG_POSE_GOAL_TOPIC = "/manipulation/debug_pose_goal";
conststr GET_JOINT_SERVICE = "/manipulation/get_joints";
conststr TOGGLE_SERVO_SERVICE = "/manipulation/toggle_servo";

#endif // MANIPULATION_CONSTANTS_CPP