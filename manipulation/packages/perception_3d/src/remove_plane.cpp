#include <climits>
#include <cstdint>
#include <iostream>
#include <memory>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <perception_3d/macros.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <variant>

#include "rclcpp/rclcpp.hpp"
// #include <frida_interfaces/action/detect_objects3_d.hpp>
#include <frida_interfaces/srv/remove_plane.hpp>
#include <frida_interfaces/srv/test.hpp>

class TableSegmentationNode : public rclcpp::Node {
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_;
  rclcpp::Service<frida_interfaces::srv::Test>::SharedPtr test_srv;
  rclcpp::Service<frida_interfaces::srv::RemovePlane>::SharedPtr
      remove_place_srv;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

public:
  TableSegmentationNode() : Node("table_segmentation_node") {
    RCLCPP_INFO(this->get_logger(), "Starting Table Segmentation Node");
    this->cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered", rclcpp::SensorDataQoS(),
        std::bind(&TableSegmentationNode::pointCloudCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud topic");

    last_ = nullptr;

    this->test_srv = this->create_service<frida_interfaces::srv::Test>(
        "/manip/test_service",
        std::bind(&TableSegmentationNode::test_service, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3));

    this->remove_place_srv =
        this->create_service<frida_interfaces::srv::RemovePlane>(
            "/manip/extract_plane",
            std::bind(&TableSegmentationNode::remove_plane, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "Service created");
  }

  void remove_plane(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<frida_interfaces::srv::RemovePlane::Request>
          request,
      const std::shared_ptr<frida_interfaces::srv::RemovePlane::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "remove_plane");
    response->health_response = u32bStatusErrorCodes::OK;
    response->cloud = sensor_msgs::msg::PointCloud2();

    if (this->last_ == nullptr) {
      response->health_response = u32bStatusErrorCodes::NO_POINTCLOUD;
      // response->cloud = NULL;
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);

    copyPointCloud(last_, cloud_out);

    response->health_response =
        this->extractPlane(cloud_out, cloud_out, request->extract_or_remove);

    if (response->health_response != u32bStatusErrorCodes::OK) {
      return;
    }

    pcl::toROSMsg(*cloud_out, response->cloud);
    response->health_response = u32bStatusErrorCodes::OK;

    return;
  }

  void test_service(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<frida_interfaces::srv::Test::Request> request,
      const std::shared_ptr<frida_interfaces::srv::Test::Response> response) {
    // std::cout << "test_service" << std::endl;
    RCLCPP_INFO(this->get_logger(), "test_service");
    response->success = u32bStatusErrorCodes::OK;

    if (this->last_ == nullptr) {
      response->success = u32bStatusErrorCodes::NO_POINTCLOUD;
      return;
    }

    std::string base_path = "/home/ivanromero/Desktop/home2/manipulation/"
                            "packages/perception_3d/pcl_debug/";
    std::string prefix = request->base_name;
    std::string filename_base = base_path + prefix;

    RCLCPP_INFO(this->get_logger(), "Saving point cloud to %s",
                filename_base.c_str());

    response->success = savePointCloud(filename_base + "_original.pcd", last_);

    ASSERT_AND_RETURN_CODE(response->success, u32bStatusErrorCodes::OK,
                           "Error saving point cloud with code %d",
                           response->success);

    RCLCPP_INFO(this->get_logger(), "Filtering point cloud");

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    //     new pcl::PointCloud<pcl::PointXYZ>);

    // cloud->points = last_->points;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);

    // response->success =
    //     this->filterPointsByHeight(0, INT_MAX, last_, cloud_out);

    // ASSERT_AND_RETURN_CODE(response->success, u32bStatusErrorCodes::OK,
    //                        "Error filtering point cloud with code %d",
    //                        response->success);

    // response->success =
    //     savePointCloud(filename_base + "_filtered.pcd", cloud_out);

    // ASSERT_AND_RETURN_CODE(response->success, u32bStatusErrorCodes::OK,
    //                        "Error saving filtered point cloud with code %d",
    //                        response->success);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(
        new pcl::PointCloud<pcl::PointXYZ>);
    response->success = this->removePlane(last_, cloud_out2);

    ASSERT_AND_RETURN_CODE(response->success, u32bStatusErrorCodes::OK,
                           "Error removing plane with code %d",
                           response->success);

    response->success =
        savePointCloud(filename_base + "_filtered_no_plane.pcd", cloud_out2);

    // auto msg = sensor_msgs::msg::PointCloud2();
    // pcl::toROSMsg(*cloud_out2, msg);
    // msg.header.frame_id = "zed_left_camera_frame";
    // this->publ->publish(msg);

    ASSERT_AND_RETURN_CODE(response->success, u32bStatusErrorCodes::OK,
                           "Error saving filtered point cloud with code %d",
                           response->success);

    response->success = u32bStatusErrorCodes::OK;

    return;
  }

  static uint32_t
  savePointCloud(const std::string filename,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 const rclcpp::Logger logger = rclcpp::get_logger("save_pcd")) {
    try {
      pcl::io::savePCDFile(filename, *cloud);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger, "Error saving point cloud: %s", e.what());
      return u32bStatusErrorCodes::COULD_NOT_SAVE_POINT_CLOUD;
    }
    return u32bStatusErrorCodes::OK;
  }

  uint32_t
  pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received point cloud");
    try {
      // RCLCPP_INFO(this->get_logger(), "Converting point cloud");
      last_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *last_);
      // RCLCPP_INFO(this->get_logger(), "Converted point cloud");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting point cloud: %s",
                   e.what());
      return u32bStatusErrorCodes::COULD_NOT_CONVERT_POINT_CLOUD;
    }
    return u32bStatusErrorCodes::OK;
  }

  uint32_t
  filterPointsByHeight(const int min_height, const int max_height,
                       _IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    uint32_t status = u32bStatusErrorCodes::OK;

    for (const auto &point : cloud->points) {
      if (point.z > min_height && point.z < max_height) {
        cloud_out->points.push_back(point);
      }
    }

    return status;
  }

  uint32_t copyPointCloud(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    *cloud_out = *cloud;
    return u32bStatusErrorCodes::OK;
  }

  uint32_t removePlane(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    uint32_t status = u32bStatusErrorCodes::OK;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not estimate a planar model");
      return u32bStatusErrorCodes::COULD_NOT_ESTIMATE_PLANAR_MODEL;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    this->copyPointCloud(cloud, cloud_out);
    extract.setInputCloud(cloud_out);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);

    pcl::ExtractIndices<pcl::PointXYZ> extract2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(
        new pcl::PointCloud<pcl::PointXYZ>);

    status = copyPointCloud(cloud, cloud_f);

    extract.setInputCloud(cloud_f);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_f);

    this->savePointCloud("/home/ivanromero/Desktop/home2/manipulation/packages/"
                         "perception_3d/pcl_debug/asdasd.pcd",
                         cloud_f);

    return status;
  };

  uint32_t extractPlane(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                        bool extract_negative = false) {
    uint32_t status = u32bStatusErrorCodes::OK;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not estimate a planar model");
      return u32bStatusErrorCodes::COULD_NOT_ESTIMATE_PLANAR_MODEL;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    this->copyPointCloud(cloud, cloud_out);
    extract.setInputCloud(cloud_out);
    extract.setIndices(inliers);
    extract.setNegative(extract_negative);
    extract.filter(*cloud_out);

    return status;
  };
};

int main(int argc, _IN_ char *argv[]) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("table_segmentation_main"),
              "Starting Table Segmentation Node");
  std::shared_ptr<rclcpp::Node> ptr = std::static_pointer_cast<rclcpp::Node>(
      std::make_shared<TableSegmentationNode>());
  RCLCPP_INFO(rclcpp::get_logger("table_segmentation_main"), "Node created");
  rclcpp::spin(ptr);
  RCLCPP_INFO(rclcpp::get_logger("table_segmentation_main"), "Node spinning");
  rclcpp::shutdown();
  RCLCPP_INFO(rclcpp::get_logger("table_segmentation_main"), "Node shutdown");
  return 0;
}