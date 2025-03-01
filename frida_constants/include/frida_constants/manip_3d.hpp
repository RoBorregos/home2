#ifndef MANIP_3D_HPP
#define MANIP_3D_HPP

#include <string>

typedef const std::string consts;

consts ZED_POINT_CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered";
consts POINT_CLOUD_TOPIC = "/point_cloud";
consts REMOVE_PC_TEST = "/manip/test_service";
consts REMOVE_PLANE_SERVICE = "/manip/extract_plane";

#endif // MANIP_3D_HPP