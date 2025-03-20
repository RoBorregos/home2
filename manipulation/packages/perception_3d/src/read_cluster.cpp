#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <frida_interfaces/srv/read_pcd_file.hpp>
#include <filesystem>

// PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

class PublishNode : public rclcpp::Node {
private:
  rclcpp::Service<frida_interfaces::srv::ReadPcdFile>::SharedPtr service_;

public:
  PublishNode() : Node("publish_node") {
    RCLCPP_INFO(this->get_logger(), "Creating PCD file reader service");
    
    // Create service endpoint for PCD file reading
    service_ = this->create_service<frida_interfaces::srv::ReadPcdFile>(
        "/read_pcd_file",
        std::bind(&PublishNode::read_pcd_file, this, 
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:
  void read_pcd_file(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Request> request,
      const std::shared_ptr<frida_interfaces::srv::ReadPcdFile::Response> response) 
  {
    RCLCPP_INFO(this->get_logger(), "Processing PCD file: %s", request->pcd_path.c_str());
    
    // Validate file existence
    if (!std::filesystem::exists(request->pcd_path)) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "File not found: %s", request->pcd_path.c_str());
        return;
    }

    // Load point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(request->pcd_path, *cloud) == -1) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", request->pcd_path.c_str());
        return;
    }

    // Convert to ROS message format
    pcl::toROSMsg(*cloud, response->cloud);
    
    // TODO: Add coordinate frame transformation here
    response->cloud.header.frame_id = "link_base";  // Default frame
    response->success = true;
    
    RCLCPP_INFO(this->get_logger(), "Successfully processed %zu points", cloud->size());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublishNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting PCD file reader service");
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}