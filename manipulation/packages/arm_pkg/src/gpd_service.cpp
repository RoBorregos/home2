#include "rclcpp/rclcpp.hpp"
#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <frida_constants/manipulation_constants_cpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <frida_interfaces/srv/grasp_detection.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_ros/transforms.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace gpd_ros2 {

class GraspDetectionService : public rclcpp::Node {
public:
  GraspDetectionService() : Node("grasp_detection_service"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_) {

    // Declare parameters
    this->declare_parameter("target_frame", "link_base");
    this->declare_parameter("pcd_default_frame", "link_base");
    this->declare_parameter("transform_timeout", 1.0);

    // Create service
    service_ = this->create_service<frida_interfaces::srv::GraspDetection>(
      GRASP_DETECTION_SERVICE, 
      std::bind(&GraspDetectionService::handle_service, this, 
                std::placeholders::_1, std::placeholders::_2));

    // Create publishers
    pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(GRASP_POINTCLOUD_TOPIC, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(GRASP_MARKER_TOPIC, 10);

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

  bool transform_cloud(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
      const std::string& source_frame,
      const builtin_interfaces::msg::Time& stamp,
      const std::string& target_frame) {
    
    try {
      if (source_frame == target_frame) return true;
      RCLCPP_INFO(this->get_logger(), "Transforming cloud from %s to %s", source_frame.c_str(), target_frame.c_str());
      RCLCPP_INFO(this->get_logger(), "Stamp for cloud tf: %d", stamp.sec);
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

  struct GripperDimensions {
    std::array<double, 3> base = {0.1, 0.1, 0.05};
    std::array<double, 3> finger = {0.02, 0.02, 0.17};
    double separation = 0.08;
  } gripper_dims_;

  visualization_msgs::msg::MarkerArray create_gripper_markers(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses,
    const std::vector<double>& scores) {

    visualization_msgs::msg::MarkerArray markers;
    
    for (size_t i = 0; i < poses.size(); ++i) {
      // Extract posestamp
      const auto& pose_stamped = poses[i];
      const auto& pose = pose_stamped.pose;

      // Create_gripper with score
      auto grip_markers = create_gripper(
          pose, 
          "grasp_" + std::to_string(i), 
          pose_stamped.header.frame_id, 
          static_cast<float>(scores[i]) // Convert to float
      );
      markers.markers.insert(markers.markers.end(), grip_markers.begin(), grip_markers.end());
    }
    
    return markers;
  }

  std::vector<visualization_msgs::msg::Marker> create_gripper(
    const geometry_msgs::msg::Pose& base_pose,
    const std::string& ns,
    const std::string& frame_id,
    float score = 0.5f) {

    std::vector<visualization_msgs::msg::Marker> markers;
    double r, g, b;

    // Map score to hue in [0.0, 0.333] (red to green)
    double hue = 0.333 * std::clamp(score, 0.0f, 1.0f); 

    hsv_to_rgb(hue, 1.0, 1.0, r, g, b);

    r = std::max(r, 0.1);
    g = std::max(g, 0.1);
    b = std::max(b, 0.1);

    Eigen::Quaterniond q(
        base_pose.orientation.w,
        base_pose.orientation.x,
        base_pose.orientation.y,
        base_pose.orientation.z
    );

    Eigen::Affine3d trans = Eigen::Translation3d(
        base_pose.position.x,
        base_pose.position.y,
        base_pose.position.z
    ) * q * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());

    // Apply offset to align gripper with the palm
    trans = trans * Eigen::Translation3d(0, 0, -0.15);

    // Create base marker
    auto base_marker = create_marker(
        frame_id,
        ns,
        0,
        visualization_msgs::msg::Marker::CUBE,
        trans.translation(),
        Eigen::Quaterniond(trans.rotation()),
        {gripper_dims_.base[0], gripper_dims_.base[1], gripper_dims_.base[2]},
        {r * 0.5f, g * 0.5f, b * 0.5f, 0.3f} // Base más oscura
    );
    markers.push_back(base_marker);

    // Create finger markers
    for (int i = 0; i < 2; ++i) {
      double x_offset = pow(-1, i) * gripper_dims_.separation / 2.0;
      Eigen::Affine3d finger_trans = trans * Eigen::Translation3d(
          x_offset,
          0,
          gripper_dims_.finger[2] / 2.0
      );

      auto finger_marker = create_marker(
          frame_id,
          ns,
          i + 1,
          visualization_msgs::msg::Marker::CUBE,
          finger_trans.translation(),
          Eigen::Quaterniond(trans.rotation()),
          {gripper_dims_.finger[0], gripper_dims_.finger[1], gripper_dims_.finger[2]},
          {r, g, b, 0.3f}
      );
      markers.push_back(finger_marker);
    }

    return markers;
  }

  // Function to convert Pose to Affine3d
  Eigen::Affine3d pose_to_affine(const geometry_msgs::msg::Pose& pose) {
    Eigen::Affine3d transform;
    transform.translation() = Eigen::Vector3d(
        pose.position.x,
        pose.position.y,
        pose.position.z
    );
    transform.linear() = Eigen::Quaterniond(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    ).toRotationMatrix();
    return transform;
  }


  visualization_msgs::msg::Marker create_marker(
    const std::string& frame_id,
    const std::string& ns,
    int id,
    int type,
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::vector<double>& scale,
    const std::array<float, 4>& color) {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();

    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();

    marker.scale.x = scale[0];
    marker.scale.y = scale[1];
    marker.scale.z = scale[2];

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    marker.lifetime = rclcpp::Duration(10, 0);

    return marker;
  }

  void hsv_to_rgb(double h, double s, double v, double& r, double& g, double& b) {
    h = std::fmod(h, 1.0);
    int i = static_cast<int>(h * 6);
    double f = h * 6 - i;
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);

    switch (i % 6) {
      case 0: r = v; g = t; b = p; break;  
      case 1: r = q; g = v; b = p; break;  
      case 2: r = p; g = v; b = t; break; 
      case 3: r = p; g = q; b = v; break;  
      case 4: r = t; g = p; b = v; break; 
      case 5: r = v; g = p; b = q; break;  
    }
  }

  void handle_service(
    const std::shared_ptr<frida_interfaces::srv::GraspDetection::Request> req,
    std::shared_ptr<frida_interfaces::srv::GraspDetection::Response> res) {

    const std::string target_frame = this->get_parameter("target_frame").as_string();
    const std::string pcd_default_frame = this->get_parameter("pcd_default_frame").as_string();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string source_frame;
    builtin_interfaces::msg::Time stamp;
    res->success = false;

    try {

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
        source_frame = pcd_default_frame;
        stamp = this->now();
      }
      else {
        res->success = false;
        res->message = "No input";
        return;
      }

      if (!transform_cloud(cloud, source_frame, stamp, target_frame)) {
        res->success = false;
        res->message = "Transform failed";
        return;
      }

      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_msg.header.stamp = this->now();
      cloud_msg.header.frame_id = target_frame;
      pcd_pub_->publish(cloud_msg);

      auto cloud_rgba = convert_to_rgba(cloud);
      Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(1, cloud_rgba->size());

      Eigen::Matrix3Xd view_points(3, 1);
      // // get view point from transform to cam
      // auto camera_frame = "camera_depth_optical_frame";
      // auto base_frame = "link_base";
      // RCLCPP_INFO(this->get_logger(), "Looking up transform from %s to %s", base_frame, camera_frame);
      // RCLCPP_INFO(this->get_logger(), "Stamp for view point tf: %d", stamp.sec);
      // TODO: This is failing due to sim giving me wrong tf timestamps I believe
      // auto transform = tf_buffer_->lookupTransform(
      //   base_frame, camera_frame, req->input_cloud.header.stamp, rclcpp::Duration::from_seconds(1.0));
      // view_points.col(0) = Eigen::Vector3d(
      //   transform.transform.translation.x,
      //   transform.transform.translation.y,
      //   transform.transform.translation.z
      // );

      gpd::util::Cloud gpd_cloud(cloud_rgba, camera_source, view_points);
      gpd::GraspDetector detector(req->cfg_path);
      
      detector.preprocessPointCloud(gpd_cloud);
      auto grasps = detector.detectGrasps(gpd_cloud);

      res->grasp_scores.reserve(grasps.size());
      for (const auto &grasp : grasps) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = target_frame;
        
        Eigen::Vector3d pos = grasp->getPosition();
        
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = pos.z();


        auto approach = grasp->getApproach();
        auto binormal = grasp->getBinormal();
        auto axis = grasp->getAxis();

        Eigen::Matrix3d rot;
        rot << approach.x(), binormal.x(), axis.x(),
               approach.y(), binormal.y(), axis.y(),
               approach.z(), binormal.z(), axis.z();

        Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
        R.block<3, 3>(0, 0) = rot;
        // to quat
        Eigen::Quaterniond quat(R.block<3, 3>(0, 0));

        

        // TODO: tempfix, I (Emiliano) believe TFs here are all messed up,
        // basically the grasp pose is (or should be) in the standard frame +X front +Y left +Z up
        // meanwhile the ee_link (the one we actually send the grasp to) is +Z front ??? up ???
        // gpd_ros and the old manip repo did some ugly stuff to make it work, but I'm not even sure if it did work or it was coincidentally working
        // so I'm just rotating the grasp pose to make our grasp X aim to Z -> X front to Z front
        // I'd advise not to lose more time on this and just move on to a new grasp detection method
        
        Eigen::Quaterniond q = quat * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());

        
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        res->grasp_poses.push_back(pose);
        res->grasp_scores.push_back(grasp->getScore());
        RCLCPP_INFO(this->get_logger(), "Grasp detected: %f", grasp->getScore());
      }

      auto marker_array = create_gripper_markers(res->grasp_poses, res->grasp_scores);      
      marker_pub_->publish(marker_array);
      RCLCPP_INFO(this->get_logger(), "Returning success");
      res->success = true;
      return;
    }
    catch (const std::exception& e) {
      res->success = false;
      res->message = std::string("Error: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Service failed: %s", e.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Service<frida_interfaces::srv::GraspDetection>::SharedPtr service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
};

} // namespace gpd_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gpd_ros2::GraspDetectionService>());
  rclcpp::shutdown();
  return 0;
}