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

  void read_pcd_file(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Request>
          request,
      const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "read_pcd_file");
    response->status = OK;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (request->optional_filename != "") {
      request->optional_filename =
          ament_index_cpp::get_package_share_directory("perception_3d") +
          "/cluster.pcd";
    }
    pcl::io::loadPCDFile<pcl::PointXYZ>(request->optional_filename, *cloud);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    response->cloud = cloud_msg;
    response->cloud.header.frame_id = "base_link";
    response->status = OK;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublishNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}