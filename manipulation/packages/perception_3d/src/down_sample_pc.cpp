#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <cmath>
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
  float small_size = 0.01f;
  float medium_size = 0.05f;
  float large_size = 0.10f;
  float small_radius = 1.5f; // 1.5m
  float medium_radius = 2.5f; // 2.5m
  float sqr_small_rad = std::pow(small_radius, 2);
  float sqr_med_rad = std::pow(medium_radius, 2);

public:
  DownSamplePointCloud() : Node("downsample_pointcloud") {
    RCLCPP_INFO(this->get_logger(), "Starting Publish Node");

    this->input_topic = this->declare_parameter("input_topic", input_topic);
    this->output_topic =
        this->declare_parameter("OutputPointCloudTopic", output_topic);
    this->small_size = this->declare_parameter("small_size", small_size);
    this->medium_size = this->declare_parameter("medium_size", medium_size);
    this->large_size = this->declare_parameter("large_size", large_size);

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
    PointCloudNS::Ptr small_cloud(new PointCloudNS);
    PointCloudNS::Ptr medium_cloud(new PointCloudNS);
    PointCloudNS::Ptr large_cloud(new PointCloudNS);
    PointCloudNS::Ptr temporal_cloud(new PointCloudNS);
    PointCloudNS::Ptr sampled_cloud(new PointCloudNS);

    pcl::fromROSMsg(*msg, *in_cloud);

    for(size_t i=0; i < in_cloud->points.size(); i++){
      const auto& pt = in_cloud->points[i];
      float sq_radius = (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if(sq_radius < sqr_small_rad) small_cloud->points.push_back(pt); 
      else if(sq_radius < sqr_med_rad) medium_cloud->points.push_back(pt);
      else large_cloud->points.push_back(pt);
    }
    //insert clouds
    insert_cloud(small_cloud, sampled_cloud, small_size);
    insert_cloud(medium_cloud, sampled_cloud, medium_size);
    insert_cloud(large_cloud, sampled_cloud, large_size);
    
    sensor_msgs::msg::PointCloud2 response;
    pcl::toROSMsg(*sampled_cloud, response);
    // response.header.frame_id = msg->header.frame_id;
    // response.header.stamp = msg->header.stamp;
    response.header.set__frame_id(msg->header.frame_id);
    response.header.set__stamp(msg->header.stamp);
    // response.header.stamp = this->now();

    publisher->publish(response);
  }
  void insert_cloud(const PointCloudNS::Ptr& input_cloud,PointCloudNS::Ptr& output_cloud,float& leaf_size){
    PointCloudNS::Ptr temporal_cloud(new PointCloudNS);
    pcl::VoxelGrid<pointCloudType> large_sor;
    large_sor.setInputCloud(input_cloud);
    large_sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    large_sor.filter(*temporal_cloud);
    output_cloud->insert(output_cloud->end(), temporal_cloud->begin(), temporal_cloud->end());
    
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