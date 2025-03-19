#include "rclcpp/rclcpp.hpp"
#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <frida_interfaces/srv/grasp_detection.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace gpd_ros2 {
class GraspDetection : public rclcpp::Node {
public:
  GraspDetection() : Node("grasp_detection") {
    service_ = this->create_service<frida_interfaces::srv::GraspDetection>(
      "detect_grasps", 
      std::bind(&GraspDetection::handle_service, this, 
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Grasp detection service ready");
  }

private:
  bool load_cloud_from_pcd(const std::string &pcd_path, pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_path, cloud) == -1) return false;
    return true;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convert_to_rgba(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*cloud, *cloud_rgba);
    return cloud_rgba;
  }

  void handle_service(
    const std::shared_ptr<frida_interfaces::srv::GraspDetection::Request> req,
    std::shared_ptr<frida_interfaces::srv::GraspDetection::Response> res) {
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // PCD file mode
    if (!req->pcd_path.empty()) {
      if (!load_cloud_from_pcd(req->pcd_path, *cloud)) {
        res->success = false;
        res->message = "Error loading PCD file";
        return;
      }
    }
    // PointCloud2 mode
    else if (!req->input_cloud.data.empty()) {
      pcl::fromROSMsg(req->input_cloud, *cloud);
    }
    else {
      res->success = false;
      res->message = "No input data provided";
      return;
    }

    // Convert to PointXYZRGBA
    auto cloud_rgba = convert_to_rgba(cloud);

    // Create camera source and view points matrices
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(1, cloud_rgba->size());
    Eigen::Matrix3Xd view_points = Eigen::Matrix3Xd::Zero(3, 1);

    // Create gpd::util::Cloud object
    gpd::util::Cloud gpd_cloud(cloud_rgba, camera_source, view_points);
    gpd::GraspDetector detector(req->cfg_path);
    
    try {
      detector.preprocessPointCloud(gpd_cloud);
      auto grasps = detector.detectGrasps(gpd_cloud);

      geometry_msgs::msg::PoseArray poses;
      for (const auto &grasp : grasps) {
        geometry_msgs::msg::Pose pose;
        Eigen::Vector3d pos = grasp->getPosition();
        Eigen::Quaterniond quat(grasp->getOrientation());
        
        pose.position.x = pos.x();
        pose.position.y = pos.y();
        pose.position.z = pos.z();
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        
        poses.poses.push_back(pose);
      }

      res->success = true;
      res->grasp_poses = poses;
    }
    catch (const std::exception& e) {
      res->success = false;
      res->message = std::string("Detection error: ") + e.what();
    }
  }

  rclcpp::Service<frida_interfaces::srv::GraspDetection>::SharedPtr service_;
};
} // namespace gpd_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gpd_ros2::GraspDetection>());
  rclcpp::shutdown();
  return 0;
}