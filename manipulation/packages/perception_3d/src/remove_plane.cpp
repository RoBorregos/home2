#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
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

#include <frida_interfaces/srv/cluster_object_from_point.hpp>
#include <frida_interfaces/srv/remove_plane.hpp>
#include <frida_interfaces/srv/remove_vertical_plane.hpp>
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
  rclcpp::Service<frida_interfaces::srv::RemoveVerticalPlane>::SharedPtr
      remove_vertical_plane_srv;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // debug point pub
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  // debug pcl pub
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  std::string point_cloud_topic = POINT_CLOUD_TOPIC;

  std::string package_path;

  double cluster_size = 0.37;

public:
  TableSegmentationNode() : Node("table_segmentation_node") {
    RCLCPP_INFO(this->get_logger(), "Starting Table Segmentation Node");
    this->point_cloud_topic =
        this->declare_parameter("point_cloud_topic", point_cloud_topic);

    this->point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/manipulation/perception_3d_debug/point", 10);

    this->cluster_size = this->declare_parameter("cluster_size", cluster_size);

    this->package_path =
        ament_index_cpp::get_package_share_directory("perception_3d");

    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    this->tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    qos.keep_last(1);
    RCLCPP_INFO(this->get_logger(),
                "Creating subscription to point cloud WITH BEST EFFORT");
    this->cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic, qos,
        std::bind(&TableSegmentationNode::pointCloudCallback, this,
                  std::placeholders::_1));

    this->cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/manipulation/perception_3d_debug/cloud", 10);

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
    this->remove_vertical_plane_srv =
        this->create_service<frida_interfaces::srv::RemoveVerticalPlane>(
            REMOVE_VERTICAL_PLANE_SERVICE,
            std::bind(&TableSegmentationNode::remove_vertical_plane, this,
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

    // defaults
    if (request->min_height == 0.0 and request->max_height == 0.0) {
      request->min_height = 0.1;
      request->max_height = 1.5;
    }

    if (request->close_point.header.frame_id == "") {
      // only filter too far from the robot
      pcl::PointXYZ req_point;
      req_point.x = 0.0;
      req_point.y = 0.0;
      req_point.z = 0.0;
      response->health_response = this->DistanceFilterFromPoint(
          cloud_out, req_point, cloud_out, 5.0, request->min_height,
          request->max_height);
    } else {
      geometry_msgs::msg::PointStamped point;
      if (request->close_point.header.frame_id != "base_link") {
        try {
          this->tf_buffer->transform(request->close_point, point, "base_link");
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Error transforming point: %s",
                       e.what());
          response->health_response = COULDNT_TRANSFORM_TO_BASE_LINK;
          return;
        }
      } else {
        point = request->close_point;
      }
      pcl::PointXYZ req_point;
      req_point.x = point.point.x;
      req_point.y = point.point.y;
      req_point.z = point.point.z;

      response->health_response = this->DistanceFilterFromPoint(
          cloud_out, req_point, cloud_out, 1.0, request->min_height,
          request->max_height);
    }

    ASSERT_AND_RETURN_CODE(response->health_response, OK,
                           "Error filtering point cloud with code %d",
                           response->health_response);

    response->health_response =
        this->extractPlane(cloud_out, cloud_out, request->extract_or_remove);

    ASSERT_AND_RETURN_CODE(response->health_response, OK,
                           "Error extracting plane with code %d",
                           response->health_response);

    response->health_response = this->largest_cluster(cloud_out, cloud_out);

    // publish
    //  publish this in debug pcl pub
    sensor_msgs::msg::PointCloud2 cloud_out_msg;
    try {
      pcl::toROSMsg(*cloud_out, cloud_out_msg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting point cloud: %s",
                   e.what());
      response->health_response = COULD_NOT_CONVERT_POINT_CLOUD;
      return;
    }
    cloud_out_msg.header.frame_id = "base_link";
    cloud_out_msg.header.stamp = this->now();
    this->cloud_pub_->publish(cloud_out_msg);

    ASSERT_AND_RETURN_CODE(response->health_response, OK,
                           "Error clustering point cloud with code %d",
                           response->health_response);

    pcl::toROSMsg(*cloud_out, response->cloud);
    response->health_response = OK;

    RCLCPP_INFO(this->get_logger(), "Publishing point cloud");
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

    // std::string base_path = "/home/ivanromero/Desktop/home2/manipulation/"
    //                         "packages/perception_3d/pcl_debug/";
    std::string base_path = this->package_path;
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

    response->success = savePointCloud("/height_filtered.pcd", cloud_out);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(
        new pcl::PointCloud<pcl::PointXYZ>);
    response->success = this->extractPlane(cloud_out, cloud_out2, true);

    ASSERT_AND_RETURN_CODE(response->success, OK,
                           "Error removing plane with code %d",
                           response->success);

    response->success = savePointCloud("/filtered_no_plane.pcd", cloud_out2);

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

  uint32_t radialFilter(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                        const float x, const float y, const float z,
                        const float radius) {
    cloud_out->points.clear();

    for (const auto &point : cloud->points) {
      float dx = point.x - x;
      float dy = point.y - y;
      float dz = point.z - z;
      float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (distance <= radius) {
        cloud_out->points.push_back(point);
      }
    }

    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;

    if (cloud_out->points.empty()) {
      return POINT_CLOUD_EMPTY;
    }

    return OK;
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

    // bool can_transform =
    //     request->point.header.frame_id == "base_link" ||
    //     this->tf_buffer->canTransform(
    //         "base_link", request->point.header.frame_id, tf2::TimePointZero);

    // if (!can_transform) {
    //   RCLCPP_ERROR(this->get_logger(), "Could not transform point cloud");
    //   response->status = COULDNT_TRANSFORM_TO_BASE_LINK;
    //   return;
    // }

    if (request->point.header.frame_id != "base_link") {
      geometry_msgs::msg::PointStamped point;
      try {
        this->tf_buffer->transform(request->point, point, "base_link");
        request->point = point;
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error transforming point: %s",
                     e.what());
        response->status = COULDNT_TRANSFORM_TO_BASE_LINK;
        return;
      }
    }
    this->point_pub_->publish(request->point);
    pcl::PointXYZ point;
    point.x = request->point.point.x;
    point.y = request->point.point.y;
    point.z = request->point.point.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius_out(
        new pcl::PointCloud<pcl::PointXYZ>);

    // radial filter, to eliminate chance of error by planes far from the point
    response->status = this->DistanceFilterFromPoint(
        last_, point, cloud_radius_out, 0.5, point.z - 0.3, point.z + 0.3);
    std::string base_path = this->package_path;
    response->status =
        savePointCloud(base_path + "/radial_filtered.pcd", cloud_radius_out);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);

    response->status =
        this->get_cloud_without_plane(cloud_radius_out, cloud_out);

    ASSERT_AND_RETURN_CODE(response->status, OK,
                           "Error getting current cloud with code %d",
                           response->status);

    response->status =
        savePointCloud(base_path + "/filtered_no_plane.pcd", cloud_out);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out5_debug(
        new pcl::PointCloud<pcl::PointXYZ>);

    response->status = this->extractPlane(cloud_out, cloud_out5_debug, false);

    ASSERT_AND_RETURN_CODE(response->status, OK,
                           "Error extracting plane with code %d",
                           response->status);

    response->status =
        savePointCloud(base_path + "/filtered_plane.pcd", cloud_out5_debug);

    response->status = savePointCloud(base_path + "/original.pcd", last_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(
        new pcl::PointCloud<pcl::PointXYZ>);

    response->status = this->clusterFromPoint(
        _IN_ cloud_out, _IN_ point, _OUT_ cloud_out2,
        _IN_ request->is_get_all_other_surrounding_objects);

    ASSERT_AND_RETURN_CODE(response->status, OK,
                           "Error clustering point with code %d",
                           response->status);

    response->status = savePointCloud(base_path + "/cluster.pcd", cloud_out2);

    ASSERT_AND_RETURN_CODE(response->status, OK,
                           "Error saving cluster point cloud with code %d",
                           response->status);

    response->cluster = sensor_msgs::msg::PointCloud2();
    try {
      pcl::toROSMsg(*cloud_out2, response->cluster);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting point cloud: %s",
                   e.what());
      response->status = COULD_NOT_CONVERT_POINT_CLOUD;
      return;
    }

    response->cluster.header.frame_id = "base_link";
    response->cluster.header.stamp = this->now();
    response->status = OK;

    RCLCPP_INFO(this->get_logger(),
                "Clustered object, sending response with status OK");
  }

  STATUS_RESPONSE DistanceFilterFromPoint(
      _IN_ const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
      _IN_ const pcl::PointXYZ point,
      _OUT_ std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_out,
      const float distance = 0.5, const float min_height = 0.1,
      const float max_height = 2.5) {

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(std::max(float(point.z - distance), min_height),
                         std::min(float(point.z + distance), max_height));

    pass.filter(*cloud_out);

    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud(cloud_out);
    pass2.setFilterFieldName("x");
    pass2.setFilterLimits(point.x - distance, point.x + distance);

    pass2.filter(*cloud_out);

    pcl::PassThrough<pcl::PointXYZ> pass3;
    pass3.setInputCloud(cloud_out);

    pass3.setFilterFieldName("y");
    pass3.setFilterLimits(point.y - distance, point.y + distance);
    pass3.filter(*cloud_out);

    // filter points closest to the robot arm in xy
    float min_distance = 0.15; // lower than this are excluded
    for (auto it = cloud_out->points.begin(); it != cloud_out->points.end();) {
      float dx = it->x;
      float dy = it->y;
      float distance = std::sqrt(dx * dx + dy * dy);
      if (distance < min_distance) {
        it = cloud_out->points.erase(it);
      } else {
        ++it;
      }
    }
    cloud_out->width = cloud_out->points.size();

    return OK;
  }

  STATUS_RESPONSE vertical_plane(
      _IN_ const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
      _OUT_ const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_out) {

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // set segmentation parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    // eps to 90 degrees
    seg.setEpsAngle(90.0 * M_PI / 180.0);
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.03);
    // segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not estimate a vertical plane");
      return COULD_NOT_ESTIMATE_PLANAR_MODEL;
    }
    RCLCPP_INFO(this->get_logger(), "Vertical plane found with %zu inliers",
                inliers->indices.size());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_out);

    RCLCPP_INFO(this->get_logger(), "Extracted vertical plane with %zu points",
                cloud_out->points.size());
    if (cloud_out->points.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not extract vertical plane");
      return COULD_NOT_ESTIMATE_PLANAR_MODEL;
    }
    // save the vertical plane to a file
    // std::string filename = this->package_path + "/vertical_plane.pcd";
    // try {
    //   pcl::io::savePCDFile(this->package_path + "/vertical_plane.pcd",
    //                        *cloud_out);
    // } catch (const std::exception &e) {
    //   RCLCPP_ERROR(this->get_logger(), "Error saving vertical plane: %s",
    //                e.what());
    //   // return COULD_NOT_SAVE_POINT_CLOUD;
    // }
    // RCLCPP_INFO(this->get_logger(), "Saved vertical plane to %s",
    //             (this->package_path + "/vertical_plane.pcd").c_str());
    // euclidean clustering get closest cluster
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_out);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.04); // 5cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_out);
    ec.extract(cluster_indices);
    if (cluster_indices.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No clusters found in vertical plane");
      return POINT_CLOUD_EMPTY;
    }

    RCLCPP_INFO(this->get_logger(), "Found %zu clusters in vertical plane",
                cluster_indices.size());
    // get the largest cluster
    // int largest_cluster_index = 0;
    // size_t largest_cluster_size = 0;
    // for (size_t i = 0; i < cluster_indices.size(); ++i) {
    //   if (cluster_indices[i].indices.size() > largest_cluster_size) {
    //     largest_cluster_size = cluster_indices[i].indices.size();
    //     largest_cluster_index = i;
    //   }
    // }
    // get closest cluster to the robot 
    int largest_cluster_index = -1;
    size_t closest_cluster_distance = 0;
    pcl::PointXYZ robot_position(0.0, 0.0, 0.5); // 0.5 meters above the ground
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
      pcl::PointXYZ cluster_center(0.0, 0.0, 0.0);
      for (const auto &index : cluster_indices[i].indices) {
        cluster_center.x += cloud_out->points[index].x;
        cluster_center.y += cloud_out->points[index].y;
        cluster_center.z += cloud_out->points[index].z;
      }
      // cluster_center.x /= cluster_indices[i].indices.size();
      // cluster_center.y /= cluster_indices[i].indices.size();
      // cluster_center.z /= cluster_indices[i].indices.size();
      float dx = cluster_center.x - robot_position.x;
      float dy = cluster_center.y - robot_position.y;
      float dz = cluster_center.z - robot_position.z;
      float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (distance < closest_cluster_distance ||
          largest_cluster_index == -1) {
        closest_cluster_distance = distance;
        largest_cluster_index = i;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Largest cluster has %zu points", 
                cluster_indices[largest_cluster_index].indices.size());
    // extract the largest cluster
    pcl::PointIndices::Ptr largest_cluster_indices(
        new pcl::PointIndices(cluster_indices[largest_cluster_index]));
    pcl::ExtractIndices<pcl::PointXYZ> extract_largest;
    extract_largest.setInputCloud(cloud_out);
    extract_largest.setIndices(largest_cluster_indices);
    extract_largest.setNegative(false);
    extract_largest.filter(*cloud_out);
    RCLCPP_INFO(this->get_logger(), "Extracted largest cluster with %zu points",
                cloud_out->points.size());
    if (cloud_out->points.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not extract largest cluster");
      return POINT_CLOUD_EMPTY;
    }

    return OK;
  }

  void remove_vertical_plane(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<frida_interfaces::srv::RemoveVerticalPlane::Request>
          request,
      const std::shared_ptr<
          frida_interfaces::srv::RemoveVerticalPlane::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "remove_vertical_plane");
    response->health_response = OK;

    if (this->last_ == nullptr) {
      response->health_response = NO_POINTCLOUD;
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);

    response->health_response = copyPointCloud(last_, cloud_out);

    ASSERT_AND_RETURN_CODE(response->health_response, OK,
                           "Error copying point cloud with code %d",
                           response->health_response);

    if (request->close_point.header.frame_id == "") {
      // only filter too far from the robot
      pcl::PointXYZ req_point;
      req_point.x = 0.0;
      req_point.y = 0.0;
      req_point.z = 0.0;
      response->health_response = this->DistanceFilterFromPoint(
          cloud_out, req_point, cloud_out, 0.3, request->close_point.point.z - 0.3,
          request->close_point.point.z + 0.3);
    } else {

      response->health_response = this->passThroughPlane(
          cloud_out, cloud_out, 0.15, 2.1, PassThroughFilterType::Z);
      ASSERT_AND_RETURN_CODE(response->health_response, OK,
                            "Error filtering point cloud by height with code %d",
                            response->health_response);
      response->health_response = this->passThroughPlane(
          cloud_out, cloud_out, -1.0, 1.0, PassThroughFilterType::Y);
      ASSERT_AND_RETURN_CODE(response->health_response, OK,
                            "Error filtering point cloud by height with code%d",
                            response->health_response);
      response->health_response = this->passThroughPlane(
          cloud_out, cloud_out, -1.0, 1.0, PassThroughFilterType::X);
      ASSERT_AND_RETURN_CODE(response->health_response, OK,
                            "Error filtering point cloud by height with code%d",
                            response->health_response);
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr vertical_plane_removed(
        new pcl::PointCloud<pcl::PointXYZ>);
    response->health_response =
        vertical_plane(cloud_out, vertical_plane_removed);
    ASSERT_AND_RETURN_CODE(response->health_response, OK,
                           "Error extracting vertical plane with code %d",
                           response->health_response);
    response->cloud = sensor_msgs::msg::PointCloud2();
    try {
      pcl::toROSMsg(*vertical_plane_removed, response->cloud);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting point cloud: %s",
                   e.what());
      response->health_response = COULD_NOT_CONVERT_POINT_CLOUD;
      return;
    }
    response->cloud.header.frame_id = "base_link";
    response->cloud.header.stamp = this->now();
    response->health_response = OK;
    RCLCPP_INFO(this->get_logger(),
                "Publishing point cloud without vertical "
                "plane with %zu points",
                vertical_plane_removed->points.size());
    this->cloud_pub_->publish(response->cloud);
    RCLCPP_INFO(this->get_logger(),
                "Published point cloud without vertical "
                "plane with %zu points",
                vertical_plane_removed->points.size());
    return;
  }

  uint32_t
  get_cloud_without_plane(_IN_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                          _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {

    uint32_t status = OK;
    // cloud_out =
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr(new
    //     pcl::PointCloud<pcl::PointXYZ>);

    status = copyPointCloud(cloud_in, cloud_out);

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
    // first check if it pointcloud has any data
    if (cloud->points.size() == 0) {
      RCLCPP_WARN(logger, "Point cloud is empty, not saving");
      return POINT_CLOUD_EMPTY;
    }
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
      sensor_msgs::msg::PointCloud2 msg2;
      // tf_listener->waitForTransform("base_link", msg->header.frame_id,
      //                               msg->header.stamp, rclcpp::Time(0),
      //                               rclcpp::Duration(5.0));

      // tf_listener->lookupTransform("base_link", msg->header.frame_id,
      //                              msg->header.stamp,
      //                              rclcpp::Duration(5.0));
      // bool can_transform = this->tf_buffer->canTransform(
      //     "base_link", msg->header.frame_id, tf2::TimePointZero);
      // if (!can_transform) {
      //   RCLCPP_ERROR(this->get_logger(), "Could not transform point
      //   cloud"); return COULD_NOT_CONVERT_POINT_CLOUD;
      // }

      pcl_ros::transformPointCloud("base_link", *msg, msg2, *this->tf_buffer);

      if (msg2.data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Point cloud is empty");
        return POINT_CLOUD_EMPTY;
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      try {

        pcl::fromROSMsg(msg2, *cloud);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting point cloud: %s",
                     e.what());
        return COULD_NOT_CONVERT_POINT_CLOUD;
      }
      if (cloud->points.size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "Point cloud is empty");
        return POINT_CLOUD_EMPTY;
      }

      last_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
          new pcl::PointCloud<pcl::PointXYZ>);

      // pcl::fromROSMsg(msg2, *last_);
      *last_ = *cloud;

      // last_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      //     new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::fromROSMsg(*msg, *last_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting point cloud: %s",
                   e.what());
      return COULD_NOT_CONVERT_POINT_CLOUD;
    }
    return OK;
  }

  /**
   * \brief Filter to get pointcloud within a certain range in the axis given
   * @param cloud: IN Point cloud to filter
   * @param cloud_out: OUT Point cloud to store the filtered point cloud
   * @param min_val: Minimum value to filter *Optional*. Default is 0.0
   * @param max_val: Maximum value to filter *Optional*. Default is 5.0
   * @param filter: Axis to filter. *Optional*. Default is Z
   */
  static uint32_t passThroughPlane(
      _IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
      const float min_val = 0.0, const float max_val = 5.0,
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
    pass.setFilterFieldName(filter_field_name);
    pass.setFilterLimits(min_val, max_val);
    pass.filter(*cloud_out);

    return OK;
  }

  uint32_t copyPointCloud(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    *cloud_out = *cloud;
    return OK;
  }

  /**
   * \brief Extracts the plane from a point cloud. If extract_negative is
   true,
   * the plane is removed from the point cloud

   * @param cloud: IN Point cloud to extract the plane from
   * @param cloud_out: OUT Point cloud to store the extracted plane
   * @param extract_negative: If true, the plane is removed from the point
   cloud
   */

  uint32_t extractPlane(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                        bool extract_negative = false) {
    uint32_t status = OK;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // set segmentation parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setEpsAngle(5.0f * (M_PI / 180.0f));
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.03);

    // segment the largest planar component from the input cloud
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

  /**
   * \brief Extracts a pointcloud cluster given a point
   * @param cloud: IN Point cloud to extract the cluster from
   * @param point: IN Point to find the cluster from
   * @param cloud_out: OUT Point cloud to store the extracted cluster
   */

  uint32_t
  clusterFromPoint(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   _IN_ const pcl::PointXYZ point,
                   _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                   _IN_ bool keep_or_remove = true) {

    // Using kdTree to get the closest point FROM the pointcloud to the point
    // given as input
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch,
                              pointNKNSquaredDistance) <= 0) {
      RCLCPP_WARN(this->get_logger(),
                  "No point detected at point x: %f y: %f z: %f", point.x,
                  point.y, point.z);
      return NO_POINT_DETECTED;
    }

    // Euclidean clustering (Make all clusters from the given point cloud)
    // (Input point agnostic)

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.025);
    ec.setMinClusterSize(10);   // Minimum number of points in a cluster
    ec.setMaxClusterSize(6000); // Maximum number of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Find which cluster our point belongs to
    int targetPointIdx = pointIdxNKNSearch[0];
    int targetClusterIdx = -1;

    // given the clusters resulting from the Euclidean clustering, find the
    // cluster that is closest to the point
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      for (const auto &idx : cluster_indices[i].indices) {
        // RCLCPP_INFO(this->get_logger(), "Cluster %lu, idx %d", i, idx);
        if (idx == targetPointIdx) {
          targetClusterIdx = i;
          break;
        }
      }
      if (targetClusterIdx >= 0)
        break;
    }

    if (targetClusterIdx < 0) {
      RCLCPP_WARN(this->get_logger(),
                  "No object to cluster at point x: %f y: %f z: %f", point.x,
                  point.y, point.z);
      return NO_OBJECT_TO_CLUSTER_AT_POINT;
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (const auto &idx : cluster_indices[targetClusterIdx].indices) {
      inliers->indices.push_back(idx);
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // extract.setNegative(true);
    extract.setNegative(keep_or_remove);
    extract.filter(*cloud_out);

    return OK;
  }

  STATUS_RESPONSE
  largest_cluster(_IN_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  _OUT_ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    // Using kdTree to get the closest point FROM the pointcloud to the point
    // given as input
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    if (kdtree.nearestKSearch(cloud->points[0], 1, pointIdxNKNSearch,
                              pointNKNSquaredDistance) <= 0) {
      RCLCPP_WARN(this->get_logger(),
                  "No point detected at point x: %f y: %f z: %f",
                  cloud->points[0].x, cloud->points[0].y, cloud->points[0].z);
      return NO_POINT_DETECTED;
    }
    // Euclidean clustering (Make all clusters from the given point cloud)
    // (Input point agnostic)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.015);
    ec.setMinClusterSize(100);   // Minimum number of points in a cluster
    ec.setMaxClusterSize(20000); // Maximum number of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    RCLCPP_INFO(this->get_logger(), "Found %lu clusters",
                cluster_indices.size());
    // Find which cluster our point belongs to
    int targetPointIdx = pointIdxNKNSearch[0];
    int targetClusterIdx = -1;
    // given the clusters resulting from the Euclidean clustering, find the
    // cluster that most dense
    int max_points = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "Cluster %lu, size %lu", i,
                  cluster_indices[i].indices.size());
      if (cluster_indices[i].indices.size() > max_points) {
        max_points = cluster_indices[i].indices.size();
        targetClusterIdx = i;
      }
    }
    if (targetClusterIdx < 0) {
      RCLCPP_WARN(this->get_logger(),
                  "No object to cluster at point x: %f y: %f z: %f",
                  cloud->points[0].x, cloud->points[0].y, cloud->points[0].z);
      return NO_OBJECT_TO_CLUSTER_AT_POINT;
    }
    // Get cluster given
    cloud_out->points.clear();
    for (const auto &idx : cluster_indices[targetClusterIdx].indices) {
      cloud_out->points.push_back(cloud->points[idx]);
    }
    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;
    RCLCPP_INFO(this->get_logger(),
                "Original cloud size: %lu, clustered %lu points",
                cloud->points.size(), cloud_out->points.size());
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