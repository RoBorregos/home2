#ifndef MANIP_3D_HPP
#define MANIP_3D_HPP

#include <string>

typedef const std::string conststr;

conststr ZED_POINT_CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered";
conststr POINT_CLOUD_TOPIC = "/point_cloud";
conststr REMOVE_PC_TEST = "/manipulation/test_service";
conststr REMOVE_PLANE_SERVICE = "/manipulation/extract_plane";
conststr CLUSTER_OBJECT_SERVICE = "/manipulation/cluster_object";
conststr ADD_PICK_PRIMITIVES_SERVICE = "/manipulation/add_pick_primitives";
conststr ADD_COLLISION_SERVICE = "/manipulation/add_collision_objects";
conststr PERCEPTION_SERVICE = "/manipulation/perception_service";

#endif // MANIP_3D_HPP