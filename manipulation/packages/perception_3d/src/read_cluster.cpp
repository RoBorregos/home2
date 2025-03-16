#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <climits>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <perception_3d/macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <variant>

// #include <tf2_ros/static_transform_broadcaster.hpp>
// #include <frida_interfaces/action/detect_objects3_d.hpp>

#include <frida_constants/manip_3d.hpp>
// #include <frida_interfaces/srv/read_pcd_file.hpp>
#include <frida_interfaces/srv/read_pcd_file.hpp>
#include <frida_interfaces/srv/remove_plane.hpp>
#include <frida_interfaces/srv/test.hpp>
#include <filesystem>

using namespace std::chrono_literals;

class PublishNode : public rclcpp::Node {
private:
  rclcpp::Service<frida_interfaces::srv::ReadPcdFile>::SharedPtr service;

public:
  PublishNode() : Node("publish_node") {
    RCLCPP_INFO(this->get_logger(), "Service created");
    this->service = this->create_service<frida_interfaces::srv::ReadPcdFile>(
        "/read_pcd_file",
        std::bind(&PublishNode::read_pcd_file, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
  };

  void read_pcd_file(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Request> request,
    const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Response> response
  ) {
    RCLCPP_INFO(this->get_logger(), "Recibida solicitud para: %s", request->pcd_path.c_str());
    std::string file_path = request->pcd_path;

    // Verificar si el archivo existe
    if (!std::filesystem::exists(file_path)) {
        response->success = false;
        RCLCPP_INFO(this->get_logger(), "El archivo no existe");
        return;
    }

    // Intentar cargar el PCD
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
        response->success = false;
        RCLCPP_INFO(this->get_logger(), "Error cargando PCD");
        return;
    }

    pcl::toROSMsg(*cloud, response->cloud);
    response->cloud.header.frame_id = "link_base";
    response->success = true;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublishNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


// using namespace std::chrono_literals;

// class PublishNode : public rclcpp::Node {
// private:
//   rclcpp::Service<frida_interfaces::srv::ReadPcdFile>::SharedPtr service;

// public:
//   PublishNode() : Node("publish_node") {
//     service = this->create_service<frida_interfaces::srv::ReadPcdFile>(
//         "/read_pcd_file",
//         [this](const std::shared_ptr<rmw_request_id_t> request_header,
//                const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Request> request,
//                const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Response> response) {
//             this->handle_service_request(request_header, request, response);
//         });
//   }

// private:
//   void handle_service_request(
//       const std::shared_ptr<rmw_request_id_t> request_header,
//       const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Request> request,
//       const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Response> response) {
    
//     RCLCPP_INFO(this->get_logger(), "Recibida solicitud para: %s", request->pcd_path.c_str());
    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
//     try {
//         // Usar el campo correcto del request
//         std::string file_path = request->pcd_path.empty() ? 
//             ament_index_cpp::get_package_share_directory("perception_3d") + "/cluster.pcd" :
//             request->pcd_path;

//         if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
//             throw std::runtime_error("Error cargando PCD");
//         }

//         pcl::toROSMsg(*cloud, response->cloud);
//         response->cloud.header.frame_id = "base_link";
//         response->success = true;
        
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
//         response->success = false;
//     }
//   }
// };

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<PublishNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
