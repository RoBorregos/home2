#include "frida_constants/manipulation_constants_cpp.hpp"
#include "rclcpp/rclcpp.hpp"
// #include <Eigen/src/Geometry/Quaternion.h>
#include <cstdint>

#include <memory>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <queue>
#include <rclcpp/client.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <frida_constants/manipulation_constants_cpp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <perception_3d/macros.hpp>

#include <frida_interfaces/msg/collision_object.h>
#include <frida_interfaces/srv/add_collision_objects.hpp>
#include <frida_interfaces/srv/add_pick_primitives.hpp>
#include <frida_interfaces/srv/get_plane_bbox.hpp>
#include <frida_interfaces/srv/remove_plane.hpp>
#include <utility>

enum class PassThroughFilterType { X, Y, Z };

struct BoxPrimitiveParams {
  pcl::PointXYZ centroid;
  long double width;
  long double height;
  long double depth;
  geometry_msgs::msg::Quaternion orientation;
  pcl::PointXYZ xy1;
  pcl::PointXYZ xy2;
  pcl::PointXYZ xy3;
  pcl::PointXYZ xy4;
};

class AddPrimitivesNode : public rclcpp::Node {
private:
  rclcpp::Service<frida_interfaces::srv::AddPickPrimitives>::SharedPtr
      add_pick_primitives_srv;
  rclcpp::Service<frida_interfaces::srv::GetPlaneBbox>::SharedPtr
      get_plane_bbox_srv;
  rclcpp::Client<frida_interfaces::srv::AddCollisionObjects>::SharedPtr
      add_collision_object_client;
  rclcpp::Client<frida_interfaces::srv::RemovePlane>::SharedPtr
      remove_plane_client;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

public:
  AddPrimitivesNode() : Node("add_primitives_node") {

    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    RCLCPP_INFO(this->get_logger(), "Starting Add Primitives Node");

    this->tf_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Creating services");

    this->add_pick_primitives_srv =
        this->create_service<frida_interfaces::srv::AddPickPrimitives>(
            ADD_PICK_PRIMITIVES_SERVICE,
            std::bind(&AddPrimitivesNode::add_pick_primitives, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));

    this->get_plane_bbox_srv =
        this->create_service<frida_interfaces::srv::GetPlaneBbox>(
            GET_PLANE_BBOX_SERVICE,
            std::bind(&AddPrimitivesNode::get_plane_bbox, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));

    this->add_collision_object_client =
        this->create_client<frida_interfaces::srv::AddCollisionObjects>(
            ADD_COLLISION_SERVICE);
    this->remove_plane_client =
        this->create_client<frida_interfaces::srv::RemovePlane>(
            REMOVE_PLANE_SERVICE);

    RCLCPP_INFO(this->get_logger(), "Service created");
  }

  STATUS_RESPONSE
  Normals(_IN_ std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
          _OUT_ BoxPrimitiveParams &box_params) {
    STATUS_RESPONSE status = OK;

    // 1. Compute centroid and covariance
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);

    // 2. Compute eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
        covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    // This line is necessary for proper orientation
    eigenVectorsPCA.col(2) =
        eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // 3. Transform the original cloud to the origin where
    // the principal components correspond to the axes
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) =
        -1.0f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected,
                             projectionTransform);

    // 4. Get the minimum and maximum points of the transformed cloud
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    const Eigen::Vector3f meanDiagonal =
        0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // 5. Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform =
        eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // 6. Save the results
    box_params.orientation.w = bboxQuaternion.w();
    box_params.orientation.x = bboxQuaternion.x();
    box_params.orientation.y = bboxQuaternion.y();
    box_params.orientation.z = bboxQuaternion.z();

    box_params.centroid.x = bboxTransform[0];
    box_params.centroid.y = bboxTransform[1];
    box_params.centroid.z = bboxTransform[2];

    box_params.width = maxPoint.x - minPoint.x;
    box_params.depth = maxPoint.y - minPoint.y;
    box_params.height = maxPoint.z - minPoint.z;

    // 7. Compute the corners
    // First, define the corners in the local coordinate system
    float halfWidth = box_params.width / 2;
    float halfDepth = box_params.depth / 2;
    float midHeight = (minPoint.z + maxPoint.z) / 2; // Use middle of Z range

    pcl::PointXYZ corner1, corner2, corner3, corner4;
    corner1.x = -halfWidth;
    corner1.y = -halfDepth;
    corner1.z = midHeight;
    corner2.x = -halfWidth;
    corner2.y = halfDepth;
    corner2.z = midHeight;
    corner3.x = halfWidth;
    corner3.y = halfDepth;
    corner3.z = midHeight;
    corner4.x = halfWidth;
    corner4.y = -halfDepth;
    corner4.z = midHeight;

    // Now transform these corners back to the original coordinate system
    Eigen::Matrix4f invProjection = projectionTransform.inverse();

    // Function to transform a point using a 4x4 matrix
    auto transformPoint = [](const pcl::PointXYZ &point,
                             const Eigen::Matrix4f &matrix) {
      Eigen::Vector4f pointVector;
      pointVector << point.x, point.y, point.z, 1;
      Eigen::Vector4f transformedPoint = matrix * pointVector;
      pcl::PointXYZ result;
      result.x = transformedPoint(0) / transformedPoint(3);
      result.y = transformedPoint(1) / transformedPoint(3);
      result.z = transformedPoint(2) / transformedPoint(3);
      return result;
    };

    box_params.xy1 = transformPoint(corner1, invProjection);
    box_params.xy2 = transformPoint(corner2, invProjection);
    box_params.xy3 = transformPoint(corner3, invProjection);
    box_params.xy4 = transformPoint(corner4, invProjection);

    return status;
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

    Eigen::Vector3f center;
    center = (max_pt.getVector3fMap() + min_pt2.getVector3fMap()) / 2;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
        covariance_matrix);
    Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();

    eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
    eigenvectors.col(0) = eigenvectors.col(1).cross(eigenvectors.col(2));
    eigenvectors.col(1) = eigenvectors.col(2).cross(eigenvectors.col(0));

    Eigen::Matrix3f temp;
    temp.col(0) = eigenvectors.col(2);
    temp.col(1) = eigenvectors.col(1);
    temp.col(2) = eigenvectors.col(0);

    eigenvectors = temp;

    Eigen::Vector3f euler_angles = (eigenvectors).eulerAngles(2, 1, 0);
    Eigen::AngleAxisf keep_Z_rot(euler_angles[0], Eigen::Vector3f::UnitZ());
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(center);
    transform.rotate(keep_Z_rot);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // // Find min and max points in the transformed space
    pcl::PointXYZ min_pt, max_pt2;
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt2);

    Eigen::Vector3f center_new =
        (max_pt2.getVector3fMap() + min_pt.getVector3fMap()) / 2;
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
    transform2.translate(center_new);
    Eigen::Affine3f transform3 = transform * transform2;

    Eigen::Quaternionf quat(keep_Z_rot);

    box_params.orientation.w = quat.w();
    box_params.orientation.x = quat.x();
    box_params.orientation.y = quat.y();
    box_params.orientation.z = quat.z();
    box_params.centroid.x = center_new[0];
    box_params.centroid.y = center_new[1];
    box_params.centroid.z = center_new[2];

    // box_params.centroid.x = min_pt.x + (max_pt2.x - min_pt.x) / 2;

    // box_params.centroid.x = min_pt2.x + (max_pt.x - min_pt2.x) / 2;
    // box_params.centroid.y = min_pt2.y + (max_pt.y - min_pt2.y) / 2;
    // box_params.centroid.z = min_pt2.z + (max_pt.z - min_pt2.z) / 2;

    pcl::PointXYZ xy1, xy2, xy3, xy4;
    const double z = (min_pt.z + max_pt2.z) / 2;
    xy1.x = min_pt.x;
    xy1.y = min_pt.y;
    xy1.z = z;
    xy2.x = min_pt.x;
    xy2.y = max_pt2.y;
    xy2.z = z;
    xy3.x = max_pt2.x;
    xy3.y = max_pt2.y;
    xy3.z = z;
    xy4.x = max_pt2.x;
    xy4.y = min_pt.y;
    xy4.z = z;

    box_params.xy1 = xy1;
    box_params.xy2 = xy2;
    box_params.xy3 = xy3;
    box_params.xy4 = xy4;
    box_params.xy1 = pcl::transformPoint(xy1, transform3);
    box_params.xy2 = pcl::transformPoint(xy2, transform3);
    box_params.xy3 = pcl::transformPoint(xy3, transform3);
    box_params.xy4 = pcl::transformPoint(xy4, transform3);
    // Eigen::Affine3f transform_inverse = transform.inverse();
    // box_params.xy1 = pcl::transformPoint(xy1, transform_inverse);
    // box_params.xy2 = pcl::transformPoint(xy2, transform_inverse);
    // box_params.xy3 = pcl::transformPoint(xy3, transform_inverse);
    // box_params.xy4 = pcl::transformPoint(xy4, transform_inverse);
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
      req2->collision_objects[0].dimensions.z = 0.025;

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

        req2->collision_objects.back().dimensions.x = 0.015;

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

  void get_plane_bbox(
      const std::shared_ptr<rmw_request_id_t> req_header,
      const std::shared_ptr<frida_interfaces::srv::GetPlaneBbox::Request>
          request,
      const std::shared_ptr<frida_interfaces::srv::GetPlaneBbox::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "get_plane_bbox");

    response->health_response = OK;

    if (request->min_height <= 0.0 && request->max_height <= 0.0) {
      response->health_response = INVALID_INPUT_FILTER;
      return;
    }

    frida_interfaces::srv::RemovePlane::Request::SharedPtr req(
        new frida_interfaces::srv::RemovePlane::Request());
    req->min_height = request->min_height;
    req->max_height = request->max_height;

    this->remove_plane_client->async_send_request(
        req,
        [this, response](
            rclcpp::Client<frida_interfaces::srv::RemovePlane>::SharedFuture
                future) {
          RCLCPP_INFO(this->get_logger(), "remove_plane");
          if (future.get()->health_response != OK) {
            response->health_response = future.get()->health_response;
            return;
          }
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
              new pcl::PointCloud<pcl::PointXYZ>);
          pcl::fromROSMsg(future.get()->cloud, *cloud_out);
          if (cloud_out->points.size() == 0) {
            response->health_response = NO_POINT_DETECTED;
            return;
          }
          RCLCPP_INFO(this->get_logger(), "Computing box primitive");
          RCLCPP_INFO(this->get_logger(), "Cloud size: %zu",
                      cloud_out->points.size());
          BoxPrimitiveParams box_params;
          try {
            RCLCPP_INFO(this->get_logger(), "Computing box primitive");
            response->health_response = RansacNormals(cloud_out, box_params);
          } catch (const std::exception &exeption) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", exeption.what());
            response->health_response = INVALID_INPUT_FILTER;
            return;
          }
          // response->health_response = RansacNormals(cloud_out, box_params);
          response->health_response = RansacNormals(cloud_out, box_params);
          ASSERT_AND_RETURN_CODE(response->health_response, OK,
                                 "Error computing box primitive with code %d",
                                 response->health_response);
          box_params.height = 0.025;
          // pcl::PointXYZ min_pt, max_pt2;
          // pcl::getMinMax3D(*cloud_out, min_pt, max_pt2);
          // box_params.centroid.x = min_pt.x + (max_pt2.x - min_pt.x) / 2;
          // box_params.centroid.y = min_pt.y + (max_pt2.y - min_pt.y) / 2;
          // box_params.centroid.z = min_pt.z + (max_pt2.z - min_pt.z) / 2;
          // box_params.width = max_pt2.x - min_pt.x;
          // box_params.depth = max_pt2.y - min_pt.y;
          RCLCPP_INFO(this->get_logger(), "Box params: %Lf %Lf %Lf",
                      box_params.width, box_params.depth, box_params.height);
          // // box_params.height = std::max(max_pt2.z - min_pt.z - 0.01, 0.01);

          response->min_x = box_params.centroid.x - box_params.width / 2;
          response->min_y = box_params.centroid.y - box_params.depth / 2;
          response->min_z = box_params.centroid.z - box_params.height / 2;
          response->max_x = box_params.centroid.x + box_params.width / 2;
          response->max_y = box_params.centroid.y + box_params.depth / 2;
          response->max_z = box_params.centroid.z + box_params.height / 2;

          // response->min_x = box_params.xy1.x;
          // response->min_y = min_pt.y;
          // response->min_z = min_pt.z;
          // response->max_x = max_pt2.x;

          if (true) {
            // publish tfs 8 corners and center
            double minn[3] = {response->min_x, response->min_y,
                              response->min_z};
            double maxx[3] = {response->max_x, response->max_y,
                              response->max_z};

            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->now();
            transform.header.frame_id = "base_link";

            transform.transform.translation.x = box_params.xy1.x;
            transform.transform.translation.y = box_params.xy1.y;
            transform.transform.translation.z = (minn[2] + maxx[2]) / 2.0;
            // transform.transform.rotation.x = 0.0;
            // transform.transform.rotation.y = 0.0;
            // transform.transform.rotation.z = 0.0;
            // transform.transform.rotation.w = 1.0;
            transform.transform.rotation = box_params.orientation;
            transform.child_frame_id = "plane_corner_1";
            RCLCPP_INFO(this->get_logger(), "Sending transform");
            this->tf_broadcaster->sendTransform(transform);
            transform.transform.translation.x = box_params.xy2.x;
            transform.transform.translation.y = box_params.xy2.y;
            transform.transform.translation.z = (minn[2] + maxx[2]) / 2.0;
            transform.transform.rotation = box_params.orientation;
            // transform.transform.rotation.x = 0.0;
            // transform.transform.rotation.y = 0.0;
            // transform.transform.rotation.z = 0.0;
            // transform.transform.rotation.w = 1.0;
            transform.child_frame_id = "plane_corner_2";
            this->tf_broadcaster->sendTransform(transform);
            transform.transform.translation.x = box_params.xy3.x;
            transform.transform.translation.y = box_params.xy3.y;
            transform.transform.translation.z = (minn[2] + maxx[2]) / 2.0;
            transform.transform.rotation = box_params.orientation;
            // transform.transform.rotation.x = 0.0;
            // transform.transform.rotation.y = 0.0;
            // transform.transform.rotation.z = 0.0;
            // transform.transform.rotation.w = 1.0;
            transform.child_frame_id = "plane_corner_3";
            this->tf_broadcaster->sendTransform(transform);
            transform.transform.translation.x = box_params.xy4.x;
            transform.transform.translation.y = box_params.xy4.y;
            transform.transform.translation.z = (minn[2] + maxx[2]) / 2.0;
            transform.transform.rotation = box_params.orientation;
            // transform.transform.rotation.x = 0.0;
            // transform.transform.rotation.y = 0.0;
            // transform.transform.rotation.z = 0.0;
            // transform.transform.rotation.w = 1.0;
            transform.child_frame_id = "plane_corner_4";
            this->tf_broadcaster->sendTransform(transform);

            // // Publish transforms for all 8 corners
            // for (int i = 0; i < 2; ++i) {
            //   for (int j = 0; j < 2; ++j) {
            //     for (int k = 0; k < 2; ++k) {
            //       std::string corner_id = "plane_corner_" + std::to_string(i)
            //       +
            //                               std::to_string(j) +
            //                               std::to_string(k);
            //       transform.child_frame_id = corner_id;

            //       double x = (i == 0) ? minn[0] : maxx[0];
            //       double y = (j == 0) ? minn[1] : maxx[1];
            //       double z = (k == 0) ? minn[2] : maxx[2];

            //       transform.transform.translation.x = x;
            //       transform.transform.translation.y = y;
            //       transform.transform.translation.z = z;

            //       // transform.transform.rotation.x = 0.0;
            //       // transform.transform.rotation.y = 0.0;
            //       // transform.transform.rotation.z = 0.0;
            //       // transform.transform.rotation.w = 1.0;
            //       transform.transform.rotation = box_params.orientation;

            //       this->tf_broadcaster->sendTransform(transform);
            //     }
            //   }
            // }

            // Publish center transform
            transform.child_frame_id = "plane_center";
            transform.transform.translation.x = (minn[0] + maxx[0]) / 2.0;
            transform.transform.translation.y = (minn[1] + maxx[1]) / 2.0;
            transform.transform.translation.z = (minn[2] + maxx[2]) / 2.0;
            transform.transform.rotation = box_params.orientation;
            this->tf_broadcaster->sendTransform(transform);
          }

          response->health_response = OK;
          return;
        });
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