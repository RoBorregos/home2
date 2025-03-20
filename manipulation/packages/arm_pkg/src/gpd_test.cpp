#include "rclcpp/rclcpp.hpp"
#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <frida_interfaces/srv/grasp_detection.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_ros/transforms.hpp>

namespace gpd_ros2 {
class GraspDetection : public rclcpp::Node {
public:
  GraspDetection() : Node("grasp_detection"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_)
  {
    // Parámetros
    this->declare_parameter("target_frame", "link_base");
    this->declare_parameter("pcd_default_frame", "link_base");
    this->declare_parameter("transform_timeout", 1.0);

    service_ = this->create_service<frida_interfaces::srv::GraspDetection>(
      "detect_grasps", 
      std::bind(&GraspDetection::handle_service, this, 
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service ready");
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

  bool transform_cloud(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
      const std::string& source_frame,
      const builtin_interfaces::msg::Time& stamp,
      const std::string& target_frame)
  {
    try {
      if (source_frame == target_frame) return true;

      const double timeout = this->get_parameter("transform_timeout").as_double();
      auto transform = tf_buffer_->lookupTransform(
        target_frame, 
        source_frame,
        stamp,
        rclcpp::Duration::from_seconds(timeout));

      Eigen::Matrix4f matrix = tf2::transformToEigen(transform.transform).matrix().cast<float>();
      pcl::transformPointCloud(*cloud, *cloud, matrix);
      return true;
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
      return false;
    }
  }

  void handle_service(
    const std::shared_ptr<frida_interfaces::srv::GraspDetection::Request> req,
    std::shared_ptr<frida_interfaces::srv::GraspDetection::Response> res) 
  {
    const std::string target_frame = this->get_parameter("target_frame").as_string();
    const std::string pcd_default_frame = this->get_parameter("pcd_default_frame").as_string();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string source_frame;
    builtin_interfaces::msg::Time stamp;

    try {
      // Control input
      if (!req->input_cloud.data.empty()) {
        pcl::fromROSMsg(req->input_cloud, *cloud);
        source_frame = req->input_cloud.header.frame_id;
        stamp = req->input_cloud.header.stamp;
      }
      else if (!req->pcd_path.empty()) {
        if (!load_cloud_from_pcd(req->pcd_path, *cloud)) {
          res->success = false;
          res->message = "PCD load failed";
          return;
        }
        source_frame = pcd_default_frame; // Use default frame
        stamp = this->now();
      }
      else {
        res->success = false;
        res->message = "No input";
        return;
      }

      // Transform to target_frame
      if (!transform_cloud(cloud, source_frame, stamp, target_frame)) {
        res->success = false;
        res->message = "Transform failed";
        return;
      }

      // Process point cloud
      auto cloud_rgba = convert_to_rgba(cloud);
      Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(1, cloud_rgba->size());
      Eigen::Matrix3Xd view_points = Eigen::Matrix3Xd::Zero(3, 1);

      gpd::util::Cloud gpd_cloud(cloud_rgba, camera_source, view_points);
      gpd::GraspDetector detector(req->cfg_path);
      
      detector.preprocessPointCloud(gpd_cloud);
      auto grasps = detector.detectGrasps(gpd_cloud);

      // Fill response
      res->grasp_scores.reserve(grasps.size());
      for (const auto &grasp : grasps) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = target_frame; // Frame consistence
        
        Eigen::Vector3d pos = grasp->getPosition();
        Eigen::Quaterniond quat(grasp->getOrientation());
        
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = pos.z();
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();

        res->grasp_poses.push_back(pose);
        res->grasp_scores.push_back(grasp->getScore());
      }

      res->success = true;
    }
    catch (const std::exception& e) {
      res->success = false;
      res->message = std::string("Error: ") + e.what();
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Service<frida_interfaces::srv::GraspDetection>::SharedPtr service_;
};
} // namespace gpd_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gpd_ros2::GraspDetection>());
  rclcpp::shutdown();
  return 0;
}