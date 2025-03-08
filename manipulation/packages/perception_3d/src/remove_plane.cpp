#include <climits>
#include <cstdint>
#include <frida_interfaces/srv/detail/cluster_object_from_point__struct.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/octree.h>
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

#include "frida_constants/manip_3d.hpp"
#include "rclcpp/rclcpp.hpp"

#include <frida_constants/manip_3d.hpp>
#include <frida_interfaces/srv/cluster_object_from_point.hpp>
#include <frida_interfaces/srv/remove_plane.hpp>
#include <frida_interfaces/srv/test.hpp>

enum class PassThroughFilterType { X, Y, Z };

class TableSegmentationNode : public rclcpp::Node {
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_;
  rclcpp::Service<frida_interfaces::srv::Test>::SharedPtr test_srv;
  rclcpp::Service<frida_interfaces::srv::RemovePlane>::SharedPtr
      remove_place_srv;
  rclcpp::Service<frida_interfaces::srv::ClusterObjectFromPoint>::SharedPtr
      cluster_object_from_point_srv;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::string point_cloud_topic = POINT_CLOUD_TOPIC;

public:
  TableSegmentationNode() : Node("table_segmentation_node") {
    RCLCPP_INFO(this->get_logger(), "Starting Table Segmentation Node");

    this->point_cloud_topic =
        this->declare_parameter("point_cloud_topic", point_cloud_topic);

    this->cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&TableSegmentationNode::pointCloudCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud topic");

    last_ = nullptr;

    this->test_srv = this->create_service<frida_interfaces::srv::Test>(
        REMOVE_PC_TEST, std::bind(&TableSegmentationNode::test_service, this,
                                  std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3));

    this->remove_place_srv =
        this->create_service<frida_interfaces::srv::RemovePlane>(
            REMOVE_PLANE_SERVICE,
            std::bind(&TableSegmentationNode::remove_plane, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));
    this->cluster_object_from_point_srv =
        this->create_service<frida_interfaces::srv::ClusterObjectFromPoint>(
            CLUSTER_OBJECT_SERVICE,
            std::bind(&TableSegmentationNode::test_cluster, this,
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
    response->health_response = OK;
    response->cloud = sensor_msgs::msg::PointCloud2();

    if (this->last_ == nullptr) {
      response->health_response = NO_POINTCLOUD;
      // response->cloud = NULL;
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);

    copyPointCloud(last_, cloud_out);

    response->health_response =
        this->extractPlane(cloud_out, cloud_out, request->extract_or_remove);

    if (response->health_response != OK) {
      return;
    }

    pcl::toROSMsg(*cloud_out, response->cloud);
    response->health_response = OK;

    return;
  }

  void test_service(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<frida_interfaces::srv::Test::Request> request,
      const std::shared_ptr<frida_interfaces::srv::Test::Response> response) {
    // std::cout << "test_service" << std::endl;
    RCLCPP_INFO(this->get_logger(), "test_service");
    response->success = OK;

    if (this->last_ == nullptr) {
      response->success = NO_POINTCLOUD;
      return;
    }

    std::string base_path = "/home/ivanromero/Desktop/home2/manipulation/"
                            "packages/perception_3d/pcl_debug/";
    std::string prefix = request->base_name;
    std::string filename_base = base_path + prefix;

    RCLCPP_INFO(this->get_logger(), "Saving point cloud to %s",
                filename_base.c_str());

    response->success = savePointCloud(filename_base + "_original.pcd", last_);

    ASSERT_AND_RETURN_CODE(response->success, OK,
                           "Error saving point cloud with code %d",
                           response->success);

    RCLCPP_INFO(this->get_logger(), "Filtering point cloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);

    RCLCPP_INFO(this->get_logger(), "Filtering point cloud by height");

    response->success = this->passThroughPlane(last_, cloud_out, 0.15, 1.5,
                                               PassThroughFilterType::Z);

    ASSERT_AND_RETURN_CODE(response->success, OK,
                           "Error filtering point cloud by height with code %d",
                           response->success);

    response->success =
        savePointCloud(filename_base + "_height_filtered.pcd", cloud_out);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(
        new pcl::PointCloud<pcl::PointXYZ>);
    response->success = this->extractPlane(cloud_out, cloud_out2, true);

    ASSERT_AND_RETURN_CODE(response->success, OK,
                           "Error removing plane with code %d",
                           response->success);

    response->success =
        savePointCloud(filename_base + "_filtered_no_plane.pcd", cloud_out2);

    ASSERT_AND_RETURN_CODE(response->success, OK,
                           "Error saving filtered point cloud with code %d",
                           response->success);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out3(
        new pcl::PointCloud<pcl::PointXYZ>);

    response->success = this->extractPlane(cloud_out, cloud_out3, false);

    ASSERT_AND_RETURN_CODE(response->success, OK,
                           "Error extracting plane with code %d",
                           response->success);

    response->success =
        savePointCloud(filename_base + "_plane.pcd", cloud_out3);

    ASSERT_AND_RETURN_CODE(response->success, OK,
                           "Error saving plane point cloud with code %d",
                           response->success);

    response->success = OK;

    return;
  }

  void test_cluster(const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<
                        frida_interfaces::srv::ClusterObjectFromPoint::Request>
                        request,
                    const std::shared_ptr<
                        frida_interfaces::srv::ClusterObjectFromPoint::Response>
                        response) {
    RCLCPP_INFO(this->get_logger(), "test_cluster");
    response->status = OK;

    if (this->last_ == nullptr) {
      response->status = NO_POINTCLOUD;
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);

    response->status = this->get_current_cloud_without_plane(cloud_out);

    ASSERT_AND_RETURN_CODE(response->status, OK,
                           "Error getting current cloud with code %d",
                           response->status);

    response->status = savePointCloud("/home/ivanromero/Desktop/home2/"
                                      "manipulation/packages/perception_3d/"
                                      "pcl_debug/cluster/filtered_no_plane.pcd",
                                      cloud_out);

    response->status = savePointCloud("/home/ivanromero/Desktop/home2/"
                                      "manipulation/packages/perception_3d/"
                                      "pcl_debug/cluster/original.pcd",
                                      last_);

    pcl::PointXYZ point;
    point.x = request->point.x;
    point.y = request->point.y;
    point.z = request->point.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(
        new pcl::PointCloud<pcl::PointXYZ>);

    response->status =
        this->clusterFromPoint(_IN_ cloud_out, _IN_ point, _OUT_ cloud_out2);

    ASSERT_AND_RETURN_CODE(response->status, OK,
                           "Error clustering point with code %d",
                           response->status);

    response->status = savePointCloud("/home/ivanromero/Desktop/home2/"
                                      "manipulation/packages/perception_3d/"
                                      "pcl_debug/cluster.pcd",
                                      cloud_out2);

    ASSERT_AND_RETURN_CODE(response->status, OK,
                           "Error saving cluster point cloud with code %d",
                           response->status);

    response->status = OK;

    return;
  }

  uint32_t get_current_cloud_without_plane(
      _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    if (this->last_ == nullptr) {
      return NO_POINTCLOUD;
    }
    uint32_t status = OK;
    cloud_out =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    status = copyPointCloud(last_, cloud_out);

    ASSERT_AND_RETURN_CODE_VALUE(
        status, OK, "Error copying point cloud with code %d", status);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(
        new pcl::PointCloud<pcl::PointXYZ>);

    status = this->passThroughPlane(cloud_out, cloud_out2, 0.15, 1.5,
                                    PassThroughFilterType::Z);

    ASSERT_AND_RETURN_CODE_VALUE(
        status, OK, "Error filtering point cloud by height with code %d",
        status);

    status = this->copyPointCloud(cloud_out2, cloud_out);

    ASSERT_AND_RETURN_CODE_VALUE(
        status, OK, "Error copying point cloud with code %d", status);

    status = this->extractPlane(cloud_out2, cloud_out, true);

    ASSERT_AND_RETURN_CODE_VALUE(status, OK,
                                 "Error removing plane with code %d", status);

    return status;
  }

  static uint32_t
  savePointCloud(const std::string filename,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 const rclcpp::Logger logger = rclcpp::get_logger("save_pcd")) {
    try {
      pcl::io::savePCDFile(filename, *cloud);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger, "Error saving point cloud: %s", e.what());
      return COULD_NOT_SAVE_POINT_CLOUD;
    }
    return OK;
  }

  uint32_t
  pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
      last_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *last_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting point cloud: %s",
                   e.what());
      return COULD_NOT_CONVERT_POINT_CLOUD;
    }
    return OK;
  }

  uint32_t passThroughPlane(
      _IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
      const float min_z = 0.0, const float max_z = 5.0,
      const PassThroughFilterType filter = PassThroughFilterType::Z) {

    std::string filter_field_name = "z";
    switch (filter) {
    case PassThroughFilterType::X:
      filter_field_name = "x";
      break;
    case PassThroughFilterType::Y:
      filter_field_name = "y";
      break;
    case PassThroughFilterType::Z:
      filter_field_name = "z";
      break;
    default:
      return INVALID_INPUT_FILTER;
      break;
    }
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*cloud_out);

    return OK;
  }

  uint32_t copyPointCloud(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    *cloud_out = *cloud;
    return OK;
  }

  uint32_t removePlane(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    uint32_t status = OK;

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
      return COULD_NOT_ESTIMATE_PLANAR_MODEL;
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
    uint32_t status = OK;

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
      return COULD_NOT_ESTIMATE_PLANAR_MODEL;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    status = this->copyPointCloud(cloud, cloud_out);

    ASSERT_AND_RETURN_CODE_VALUE(
        status, OK, "Error copying point cloud with code %d", status);

    extract.setInputCloud(cloud_out);
    extract.setIndices(inliers);
    extract.setNegative(extract_negative);
    extract.filter(*cloud_out);

    return status;
  };

  uint32_t
  clusterFromPoint(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   _IN_ const pcl::PointXYZ point,
                   _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    // https://stackoverflow.com/questions/60379640

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.1);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    if (octree.nearestKSearch(point, 1, pointIdxNKNSearch,
                              pointNKNSquaredDistance) <= 0) {
      RCLCPP_WARN(this->get_logger(),
                  "No point detected at point x: %f y: %f z: %f", point.x,
                  point.y, point.z);
      return NO_POINT_DETECTED;
    }

    cloud_out =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> pointIdxVec;

    if (octree.voxelSearch(cloud->points[pointIdxNKNSearch[0]], pointIdxVec) <=
        0) {
      RCLCPP_WARN(this->get_logger(),
                  "No object to cluster at point x: %f y: %f z: %f", point.x,
                  point.y, point.z);
      return NO_OBJECT_TO_CLUSTER_AT_POINT;
    }

    for (unsigned long i = 0; i < pointIdxVec.size(); ++i) {
      cloud_out->points.push_back(cloud->points[pointIdxVec[i]]);
    }

    RCLCPP_INFO(this->get_logger(), "Clustered %lu points",
                cloud_out->points.size());

    return OK;
  }

  ~TableSegmentationNode() {
    RCLCPP_INFO(this->get_logger(), "Destroying Table Segmentation Node");
  }
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