#include "rclcpp/rclcpp.hpp"
#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <frida_interfaces/srv/grasp_detection.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace gpd_ros2 {
class GraspDetection : public rclcpp::Node {
public:
  GraspDetection() : Node("grasp_detection") {
    grasp_detect_serv = this->create_service<frida_interfaces::srv::GraspDetection>(
      "detect_grasps", std::bind(&GraspDetection::service_callback, this, std::placeholders::_1, std::placeholders::_2));

      RCLCPP_INFO(this->get_logger(), "Grasp pose detection server ready. Call /detect_grasps and provide the path to the PCD file to detect grasps.");
  }

private:
  bool doFileExists(const std::string &file_name) {
    std::ifstream file(file_name);
    return file.good();
  }

  void service_callback(
    const std::shared_ptr<frida_interfaces::srv::GraspDetection::Request> request,
    std::shared_ptr<frida_interfaces::srv::GraspDetection::Response> response) {
    
    std::string cfg_path = request->cfg_path;
    std::string pcd_filename = request->pcd_path;

    if (!doFileExists(cfg_path)) {
        RCLCPP_ERROR(this->get_logger(), "Configuration file not found: %s", cfg_path.c_str());
        response->success = false;
        response->message = "Configuration file not found";
        return;
    }

    if (!doFileExists(pcd_filename)) {
        RCLCPP_ERROR(this->get_logger(), "PCD file not found: %s", pcd_filename.c_str());
        response->success = false;
        response->message = "PCD file not found";
        return;
    }

    gpd::util::Cloud cloud(pcd_filename, Eigen::Matrix3Xd::Zero(3, 1));
    if (cloud.getCloudOriginal()->size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "Point cloud is empty or could not be loaded");
        response->success = false;
        response->message = "Point cloud is empty or invalid";
        return;
    }

    gpd::GraspDetector detector(cfg_path);
    detector.preprocessPointCloud(cloud);

    geometry_msgs::msg::PoseArray grasp_poses_msg;
    auto grasps = detector.detectGrasps(cloud);

    if (grasps.empty()) {
        RCLCPP_WARN(this->get_logger(), "No grasp poses detected.");
        response->success = false;
        response->message = "No grasp poses found.";
        return;
    }

    for (const auto &grasp : grasps) {
        geometry_msgs::msg::Pose pose;

        Eigen::Vector3d position = grasp->getPosition();
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();

        Eigen::Matrix3d orientation_matrix = grasp->getOrientation();
        Eigen::Quaterniond orientation(orientation_matrix);
        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();
        pose.orientation.w = orientation.w();

        grasp_poses_msg.poses.push_back(pose);
    }

    response->success = true;
    response->message = "Grasp poses detected successfully";
    response->grasp_poses = grasp_poses_msg;
  }

  // rclcpp::Service<arm_pkg::srv::GraspDetection>::SharedPtr grasp_detect_serv;
  rclcpp::Service<frida_interfaces::srv::GraspDetection>::SharedPtr grasp_detect_serv;
};
} // namespace gpd_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<GraspDetection>());
  rclcpp::spin(std::make_shared<gpd_ros2::GraspDetection>());
  rclcpp::shutdown();
  return 0;
}