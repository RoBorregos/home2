#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <climits>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <perception_3d/macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/timer.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <variant>

#include <frida_constants/manipulation_constants_cpp.hpp>

using namespace std::chrono_literals;

typedef pcl::PointXYZ pointCloudType; // No RGB
// typedef pcl::PointXYZRGB pointCloudType; // RGB
typedef pcl::PointCloud<pointCloudType> PointCloudNS;

class DownSamplePointCloud : public rclcpp::Node {
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;

  std::string input_topic = ZED_POINT_CLOUD_TOPIC;
  std::string output_topic = POINT_CLOUD_TOPIC;
  float leaf_size = 0.01f;

public:
  DownSamplePointCloud() : Node("downsample_pointcloud") {
    RCLCPP_INFO(this->get_logger(), "Starting Publish Node");

    this->input_topic = this->declare_parameter("input_topic", input_topic);
    this->output_topic =
        this->declare_parameter("OutputPointCloudTopic", output_topic);
    this->leaf_size = this->declare_parameter("leaf_size", leaf_size);

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

    this->publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, qos);

    this->subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(),
        std::bind(&DownSamplePointCloud::pointcloud_callback, this,
                  std::placeholders::_1));
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    PointCloudNS::Ptr in_cloud(new PointCloudNS);
    PointCloudNS::Ptr sampled_cloud(new PointCloudNS);

    pcl::fromROSMsg(*msg, *in_cloud);

    pcl::VoxelGrid<pointCloudType> sor;
    sor.setInputCloud(in_cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*sampled_cloud);

    sensor_msgs::msg::PointCloud2 response;
    pcl::toROSMsg(*sampled_cloud, response);
    // response.header.frame_id = msg->header.frame_id;
    // response.header.stamp = msg->header.stamp;
    response.header.set__frame_id(msg->header.frame_id);
    response.header.set__stamp(msg->header.stamp);
    // response.header.stamp = this->now();

    publisher->publish(response);
  }
};

int main(int argc, _IN_ char *argv[]) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("downsample_pointcloud_main"),
              "Starting downsampling pointcloud node");

  rclcpp::spin(std::make_shared<DownSamplePointCloud>());

  rclcpp::shutdown();

  return 0;
}