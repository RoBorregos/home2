#include "rclcpp/rclcpp.hpp"
#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <frida_interfaces/srv/grasp_detection.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace gpd_ros2 {
class GraspDetection : public rclcpp::Node {
public:
  GraspDetection() : Node("grasp_detection") {
    service_ = this->create_service<frida_interfaces::srv::GraspDetection>(
      "detect_grasps", 
      std::bind(&GraspDetection::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Servicio de detección de agarres listo");
  }

private:
  bool file_exists(const std::string &path) {
    std::ifstream file(path);
    return file.good();
  }

  void handle_service(
    const std::shared_ptr<frida_interfaces::srv::GraspDetection::Request> req,
    std::shared_ptr<frida_interfaces::srv::GraspDetection::Response> res) {
    
    if (!file_exists(req->cfg_path)) {
        RCLCPP_ERROR(this->get_logger(), "Archivo de configuración no encontrado: %s", req->cfg_path.c_str());
        res->success = false;
        res->message = "Configuración no encontrada";
        return;
    }

    if (!file_exists(req->pcd_path)) {
        RCLCPP_ERROR(this->get_logger(), "Archivo PCD no encontrado: %s", req->pcd_path.c_str());
        res->success = false;
        res->message = "PCD no encontrado";
        return;
    }

    // Procesar nube de puntos
    gpd::util::Cloud cloud(req->pcd_path, Eigen::Matrix3Xd::Zero(3, 1));
    if (cloud.getCloudOriginal()->size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "Point cloud is empty or could not be loaded");
        res->success = false;
        res->message = "Point cloud is empty or invalid";
        return;
    }

    // Detectar agarres
    gpd::GraspDetector detector(req->cfg_path);
    detector.preprocessPointCloud(cloud);

    auto grasps = detector.detectGrasps(cloud);

    // Construir respuesta
    if (grasps.empty()) {
        RCLCPP_WARN(this->get_logger(), "No grasp poses detected.");
        res->success = false;
        res->message = "No grasp poses found.";
        return;
    }

    geometry_msgs::msg::PoseArray poses;
    for (const auto &grasp : grasps) {
        geometry_msgs::msg::Pose pose;
        Eigen::Vector3d pos = grasp->getPosition();
        Eigen::Matrix3d rot = grasp->getOrientation();
        Eigen::Quaterniond quat(rot);
        
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
    res->message = "Grasp poses detected successfully.";
    res->grasp_poses = poses;
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