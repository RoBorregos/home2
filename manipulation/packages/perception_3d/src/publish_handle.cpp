#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <frida_interfaces/srv/detail/remove_vertical_plane__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <pcl-1.12/pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <climits>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <variant>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <frida_constants/manipulation_constants_cpp.hpp>
#include <perception_3d/macros.hpp>

#include <yaml-cpp/yaml.h>

// #include <frida_interfaces/srv/cluster_object_from_point.hpp>
// #include <frida_interfaces/srv/remove_plane.hpp>
#include <frida_interfaces/srv/remove_vertical_plane.hpp>
// #include <frida_interfaces/srv/test.hpp>

using namespace std::chrono_literals;

template <typename T>
std::future_status wait_for_future_with_timeout(
    typename rclcpp::Client<T>::FutureAndRequestId &future,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::milliseconds timeout = 500ms) {
  auto status = future.wait_for(timeout);
  auto start_time = std::chrono::steady_clock::now();
  while (status != std::future_status::ready) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
    status = future.wait_for(10ms);
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time > timeout) {
      return status;
    }
  }
  return status;
}

class CallServicesNode : public rclcpp::Node {
public:
  rclcpp::Client<frida_interfaces::srv::RemoveVerticalPlane>::SharedPtr
      remove_vertical_plane_client;
  CallServicesNode() : Node("callservicesnode") {
    RCLCPP_INFO(this->get_logger(), "Starting CallServicesNode");
    this->remove_vertical_plane_client =
        this->create_client<frida_interfaces::srv::RemoveVerticalPlane>(
            REMOVE_VERTICAL_PLANE_SERVICE);
    RCLCPP_INFO(this->get_logger(),
                "Remove Vertical Plane Client created, waiting for services");
    this->remove_vertical_plane_client->wait_for_service(
        std::chrono::seconds(30));
    RCLCPP_INFO(this->get_logger(), "Remove Vertical Plane Service ready");
  }
};
class PublishHandleA : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<CallServicesNode> call_services_node;
  geometry_msgs::msg::PoseStamped pose_;
  tf2_ros::Buffer::SharedPtr tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer;

  double rel_x_, rel_y_, rel_yaw_;

public:
  PublishHandleA(std::shared_ptr<CallServicesNode> call_services_node)
      : Node("publish_handle_node"), call_services_node(call_services_node) {
    RCLCPP_INFO(this->get_logger(), "Starting PublishHandleA Node");

    // Load relative pose from YAML
    std::string config_path =
        ament_index_cpp::get_package_share_directory("perception_3d") +
        "/config/relative_docking_pose.yaml";
    YAML::Node config = YAML::LoadFile(config_path);
    this->rel_x_ = config["x"].as<double>();
    this->rel_y_ = config["y"].as<double>();
    this->rel_yaw_ = config["yaw"].as<double>();

    this->pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/door_handle_pose", 10);
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    this->timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PublishHandleA::timer_callback, this));
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Publishing pose");
    auto fut =
        this->call_services_node->remove_vertical_plane_client
            ->async_send_request(
                std::make_shared<
                    frida_interfaces::srv::RemoveVerticalPlane::Request>());
    auto res = wait_for_future_with_timeout<
        frida_interfaces::srv::RemoveVerticalPlane>(
        fut, this->call_services_node->get_node_base_interface(), 500ms);
    if (res == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(),
                  "Remove Vertical Plane Service call succeeded");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Remove Vertical Plane Service call timed out");
    }
    frida_interfaces::srv::RemoveVerticalPlane::Response::SharedPtr result =
        fut.get();

    if (result->health_response != 0) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Remove Vertical Plane Service call failed with error code: %d",
          result->health_response);
      return;
    }

    pcl::PointXYZ min_pt, max_pt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    // transform to map frame
    if (!this->tf_buffer_->canTransform("map", result->cloud.header.frame_id,
                                        result->cloud.header.stamp)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Cannot transform point cloud to map frame");
      return;
    }
    geometry_msgs::msg::TransformStamped transform_stamped =
        this->tf_buffer_->lookupTransform("map", result->cloud.header.frame_id,
                                          result->cloud.header.stamp);

    // Convert ROS message to PCL point cloud
    pcl::fromROSMsg(result->cloud, *cloud);

    // Transform the point cloud
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    Eigen::Affine3d eigen_transform =
        tf2::transformToEigen(transform_stamped.transform);
    pcl::transformPointCloud(*cloud, transformed_cloud, eigen_transform);
    RCLCPP_INFO(this->get_logger(), "Point cloud transformed to map frame");

    pcl::getMinMax3D(transformed_cloud, min_pt, max_pt);

    RCLCPP_INFO(this->get_logger(),
                "Min point: (%f, %f, %f), Max point: (%f, %f, %f)", min_pt.x,
                min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);
    // Create a pose from the min and max points
    this->pose_.header.frame_id = "map";
    // this->pose_.header.stamp = this->now();
    this->pose_.header.stamp = result->cloud.header.stamp;
    this->pose_.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
    this->pose_.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
    // this->pose_.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
    // this->pose_.pose.position.z = 0.0; // Assuming a 2D plane, set z to 0
    // this->pose_.pose.orientation.x = 0.0;
    // this->pose_.pose.orientation.y = 0.0;
    // this->pose_.pose.orientation.z = 0.0;
    // this->pose_.pose.orientation.w = 1.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, rel_yaw_);
    tf2::Vector3 offset(rel_x_, rel_y_, 0.0);

    // Rotate offset by handle orientation (identity for now)
    tf2::Vector3 rotated_offset = tf2::quatRotate(q, offset);
    this->pose_.pose.position.x =
        this->pose_.pose.position.x + rotated_offset.x();
    this->pose_.pose.position.y =
        this->pose_.pose.position.y + rotated_offset.y();

    tf2::Quaternion base_orientation;
    base_orientation.setRPY(0, 0, rel_yaw_);
    this->pose_.pose.orientation = tf2::toMsg(base_orientation);

    this->pose_pub_->publish(this->pose_);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<PublishHandleA>(std::make_shared<CallServicesNode>());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}