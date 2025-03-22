#include "frida_constants/manip_3d.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <memory>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <frida_constants/manip_3d.hpp>
#include <perception_3d/macros.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <frida_interfaces/msg/collision_object.h>
#include <frida_interfaces/srv/add_collision_objects.hpp>
#include <frida_interfaces/srv/add_pick_primitives.hpp>

struct BoxPrimitiveParams {
  pcl::PointXYZ centroid;
  long double width;
  long double height;
  long double depth;
  geometry_msgs::msg::Quaternion orientation;
};

class AddPrimitivesNode : public rclcpp::Node {
private:
  rclcpp::Service<frida_interfaces::srv::AddPickPrimitives>::SharedPtr
      add_pick_primitives_srv;
  rclcpp::Client<frida_interfaces::srv::AddCollisionObjects>::SharedPtr
      add_collision_object_client;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

public:
  AddPrimitivesNode() : Node("add_primitives_node") {

    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    this->add_pick_primitives_srv =
        this->create_service<frida_interfaces::srv::AddPickPrimitives>(
            ADD_PICK_PRIMITIVES_SERVICE,
            std::bind(&AddPrimitivesNode::add_pick_primitives, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));

    this->add_collision_object_client =
        this->create_client<frida_interfaces::srv::AddCollisionObjects>(
            ADD_COLLISION_SERVICE);

    RCLCPP_INFO(this->get_logger(), "Service created");
  }

  STATUS_RESPONSE
  RansacNormals(_IN_ std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
                _OUT_ BoxPrimitiveParams &box_params) {
    STATUS_RESPONSE status = OK;

    Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f centroid;

    pcl::compute3DCentroid(*cloud, centroid);

    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance_matrix);

    pcl::PointXYZ min_pt2, max_pt;
    pcl::getMinMax3D(*cloud, min_pt2, max_pt);

    box_params.centroid.x = min_pt2.x + (max_pt.x - min_pt2.x) / 2;
    box_params.centroid.y = min_pt2.y + (max_pt.y - min_pt2.y) / 2;
    box_params.centroid.z = min_pt2.z + (max_pt.z - min_pt2.z) / 2;

    Eigen::Vector3f center;
    center[0] = box_params.centroid.x;
    center[1] = box_params.centroid.y;
    center[2] = box_params.centroid.z;

    // Compute eigenvectors and eigenvalues of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
        covariance_matrix);
    Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();

    eigenvectors.col(2) = (eigenvectors.col(0).cross(eigenvectors.col(1))).normalized();
    eigenvectors.col(0) = (eigenvectors.col(1).cross(eigenvectors.col(2))).normalized();
    eigenvectors.col(1) = (eigenvectors.col(2).cross(eigenvectors.col(0))).normalized();

    Eigen::Matrix3f eigenVectorsPCA1;
    eigenVectorsPCA1.col(0) = eigenvectors.col(2);
    eigenVectorsPCA1.col(1) = eigenvectors.col(1);
    eigenVectorsPCA1.col(2) = eigenvectors.col(0);
    eigenvectors = eigenVectorsPCA1;

    Eigen::Vector3f ea = (eigenvectors).eulerAngles(2, 1, 0);
    // Eigen::AngleAxisf keep_Z_Rot(ea[0], Eigen::Vector3f::UnitZ());
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    transform.translate(center);
    // transform.rotate(keep_Z_Rot);
    // Convert eigenvectors (rotation matrix) to quaternion and normalize it
    Eigen::Quaternionf quat(eigenvectors);
    quat.normalize();

    // Rotate using the quaternion directly
    transform.rotate(quat);

    // Set the orientation in the box parameters
    box_params.orientation.x = quat.x();
    box_params.orientation.y = quat.y();
    box_params.orientation.z = quat.z();
    box_params.orientation.w = quat.w();

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, transform);

    // // Find min and max points in the transformed space
    pcl::PointXYZ min_pt, max_pt2;
    pcl::getMinMax3D(transformed_cloud, min_pt, max_pt2);

    box_params.width = max_pt2.x - min_pt.x;
    box_params.depth = max_pt2.y - min_pt.y;
    box_params.height = std::max(max_pt2.z - min_pt.z - 0.01, 0.01);

    return status;
  }

  STATUS_RESPONSE DownSampleObject(
      _IN_ const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
      _OUT_ std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_out) {

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.03, 0.03, 0.03);
    sor.filter(*cloud_out);
    return OK;
  }

  void add_pick_primitives(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<frida_interfaces::srv::AddPickPrimitives::Request>
          request,
      const std::shared_ptr<frida_interfaces::srv::AddPickPrimitives::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "add_pick_primitives");
    response->status = OK;

    ASSERT_AND_RETURN_CODE(request->is_object != request->is_plane, true,
                           "Object and plane cannot be true at the same time, "
                           "returning with code %d",
                           INVALID_INPUT);

    if (request->cloud.header.frame_id != "base_link" &&
        !tf_buffer->canTransform("base_link", request->cloud.header.frame_id,
                                 tf2::TimePointZero)) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform point cloud");
      response->status = COULDNT_TRANSFORM_TO_BASE_LINK;
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;

    if (request->cloud.header.frame_id != "base_link") {
      pcl_ros::transformPointCloud("base_link", request->cloud, cloud_msg,
                                   *tf_buffer);
    } else {
      cloud_msg = request->cloud;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *cloud);

    if (cloud->points.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Point cloud is empty");
      response->status = NO_POINT_DETECTED;
      return;
    }

    if (request->is_plane) {
      RCLCPP_INFO(this->get_logger(), "Adding plane primitive");

      BoxPrimitiveParams box_params;

      STATUS_RESPONSE status = RansacNormals(cloud, box_params);

      ASSERT_AND_RETURN_CODE(
          status, OK, "Error computing box primitive with code %d", status);

      std::shared_ptr<frida_interfaces::srv::AddCollisionObjects::Request>
          req2 = std::make_shared<
              frida_interfaces::srv::AddCollisionObjects::Request>();

      req2->collision_objects.resize(1);
      req2->collision_objects[0].id = "plane";
      req2->collision_objects[0].type = "box";

      req2->collision_objects[0].pose.header.frame_id = "base_link";

      req2->collision_objects[0].pose.header.stamp = this->now();

      req2->collision_objects[0].pose.pose.position.x = box_params.centroid.x;
      req2->collision_objects[0].pose.pose.position.y = box_params.centroid.y;
      req2->collision_objects[0].pose.pose.position.z = box_params.centroid.z;

      req2->collision_objects[0].pose.pose.orientation = box_params.orientation;

      req2->collision_objects[0].dimensions.x = box_params.width;
      req2->collision_objects[0].dimensions.y = box_params.depth;
      req2->collision_objects[0].dimensions.z = box_params.height;

      auto res = this->add_collision_object_client->async_send_request(
          req2,
          [this](rclcpp::Client<frida_interfaces::srv::AddCollisionObjects>::
                     SharedFuture future) {
            RCLCPP_INFO(this->get_logger(), "add_collision_object");
          });

      RCLCPP_INFO(this->get_logger(), "Plane primitive added");
      // res.wait();e

      RCLCPP_INFO(this->get_logger(), "Plane primitive added");
    } else {
      RCLCPP_INFO(this->get_logger(), "Adding object primitive");

      std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_downsampled(
          new pcl::PointCloud<pcl::PointXYZ>);
      STATUS_RESPONSE status = DownSampleObject(cloud, cloud_downsampled);

      ASSERT_AND_RETURN_CODE(status, OK,
                             "Error downsampling object with code %d", status);

      std::shared_ptr<frida_interfaces::srv::AddCollisionObjects::Request>
          req2 = std::make_shared<
              frida_interfaces::srv::AddCollisionObjects::Request>();

      for (pcl::PointXYZ point : cloud_downsampled->points) {

        req2->collision_objects.push_back(
            frida_interfaces::msg::CollisionObject());

        req2->collision_objects.back().id =
            "frida_pick_object_ " + std::to_string(point.x) + " " +
            std::to_string(point.y) + " " + std::to_string(point.z);
        req2->collision_objects.back().type = "sphere";

        req2->collision_objects.back().pose.header.frame_id = "base_link";
        req2->collision_objects.back().pose.header.stamp = this->now();

        req2->collision_objects.back().pose.pose.position.x = point.x;
        req2->collision_objects.back().pose.pose.position.y = point.y;
        req2->collision_objects.back().pose.pose.position.z = point.z;

        req2->collision_objects.back().dimensions.x = 0.01;

        req2->collision_objects.back().pose.pose.orientation.x = 0;
        req2->collision_objects.back().pose.pose.orientation.y = 0;
        req2->collision_objects.back().pose.pose.orientation.z = 0;
        req2->collision_objects.back().pose.pose.orientation.w = 1;
      }

      auto res = this->add_collision_object_client->async_send_request(
          req2,
          [this](rclcpp::Client<frida_interfaces::srv::AddCollisionObjects>::
                     SharedFuture future) {
            RCLCPP_INFO(this->get_logger(), "add_collision_object");
          });

      RCLCPP_INFO(this->get_logger(), "Object primitives added");
    }
    response->status = OK;
  }
};

int main(int argc, _IN_ char *argv[]) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("add_primitives_main"),
              "Starting Add Primitives Node");

  rclcpp::spin(std::make_shared<AddPrimitivesNode>());

  rclcpp::shutdown();
  return 0;
}