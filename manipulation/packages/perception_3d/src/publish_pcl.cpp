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
#include <rclcpp/timer.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <variant>

// #include <tf2_ros/static_transform_broadcaster.hpp>
// #include <frida_interfaces/action/detect_objects3_d.hpp>

#include "rclcpp/rclcpp.hpp"
#include <frida_constants/manip_3d.hpp>
#include <frida_interfaces/srv/remove_plane.hpp>
#include <frida_interfaces/srv/test.hpp>

using namespace std::chrono_literals;

class PublishNode : public rclcpp::Node {
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr timer_2;

  std::string base_link = "base_link";
  std::string child_frame = "table";
  std::string point_cloud_topic = POINT_CLOUD_TOPIC;
  std::string cloud_file = "/home/ivanromero/Desktop/home2/manipulation/"
                           "packages/perception_3d/pcl_debug/pcl_table.pcd";

public:
  PublishNode() : Node("publish_node") {
    RCLCPP_INFO(this->get_logger(), "Starting Publish Node");

    this->base_link = this->declare_parameter("base_link", base_link);
    this->child_frame =
        this->declare_parameter("zed_camera_frame", child_frame);
    this->point_cloud_topic =
        this->declare_parameter("point_cloud_topic", point_cloud_topic);
    this->cloud_file = this->declare_parameter("cloud_file", cloud_file);

    cloud =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile(this->cloud_file, *cloud) == -1) {
      PCL_ERROR("Couldn't read file %s \n", this->cloud_file.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded point cloud");

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud", qos,
        std::bind(&PublishNode::pointcloud_callback, this,
                  std::placeholders::_1));

    publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        this->point_cloud_topic, qos);

    RCLCPP_INFO(this->get_logger(), "Created publisher");

    // this->timer =
    // this->create_wall_timer(10ms, std::bind(&PublishNode::loop, this));

    RCLCPP_INFO(this->get_logger(), "Created timer");
    // this->timer_2 = this->create_wall_timer(
    //     10ms, std::bind(&PublishNode::tf_publisher, this));
  }

  void loop() {
    auto msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*cloud, msg);

    // msg.header.set__frame_id("base_link");
    msg.header.set__frame_id(this->base_link);
    // msg.header.frame_id = this->base_link;
    msg.header.set__stamp(this->now());
    // msg.header.frame_id = "base_link";
    publisher->publish(msg);
  }

  void tf_publisher() {
    int x = 0;
    int y = 0;
    int z = 0;

    auto base_link = std::make_shared<tf2_ros::TransformBroadcaster>(
        this->shared_from_this());

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    // transform_stamped.header.frame_id = this->base_link;
    transform_stamped.header.frame_id = "base_link";
    // transform_stamped.header.frame_id = "zed_left_camera_frame";
    // transform_stamped.header.frame_id = "zed_left_camera_frame";
    transform_stamped.transform.translation.x = x;
    transform_stamped.transform.translation.y = y;
    transform_stamped.transform.translation.z = z;
    transform_stamped.transform.rotation.x = 0;
    transform_stamped.transform.rotation.y = 0;
    transform_stamped.transform.rotation.z = 0;
    transform_stamped.transform.rotation.w = 1;
    transform_stamped.child_frame_id = "zed_left_camera_frame";
    // transform_stamped.child_frame_id = this->child_frame;

    base_link->sendTransform(transform_stamped);
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received point cloud");
    // pcl::fromROSMsg(*msg, *cloud);
    // if (cloud == nullptr || cloud->points.size() > 0) {
    //   cloud->points.clear();
    // }
    auto new_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*cloud, new_msg);
    // print new_msg header
    RCLCPP_INFO(this->get_logger(), "Header frame id: %s",
                new_msg.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Header stamp: %d",
                new_msg.header.stamp.sec);

    // new_msg.header.frame_id = msg->header.frame_id.c_str();
    new_msg.header.frame_id = "base_link";
    new_msg.header.stamp = msg->header.stamp;
    // new_msg.header.stamp =
    publisher->publish(new_msg);
  }
};

int main(int argc, _IN_ char *argv[]) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("table_segmentation_main"),
              "Starting Table Segmentation Node");

  rclcpp::spin(std::make_shared<PublishNode>());

  rclcpp::shutdown();

  return 0;
}