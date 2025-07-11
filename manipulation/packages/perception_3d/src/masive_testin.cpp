#include "rclcpp/rclcpp.hpp"

#include <future>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/service.hpp>
#include <variant>

#include <frida_interfaces/srv/add_pick_primitives.hpp>
#include <frida_interfaces/srv/cluster_object_from_point.hpp>
#include <frida_interfaces/srv/pick_perception_service.hpp>
#include <frida_interfaces/srv/place_perception_service.hpp>
#include <frida_interfaces/srv/remove_plane.hpp>

#include <frida_constants/manipulation_constants_cpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <perception_3d/macros.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
  rclcpp::Client<frida_interfaces::srv::AddPickPrimitives>::SharedPtr
      add_pick_primitives_client;
  rclcpp::Client<frida_interfaces::srv::RemovePlane>::SharedPtr
      remove_plane_client;
  rclcpp::Client<frida_interfaces::srv::ClusterObjectFromPoint>::SharedPtr
      cluster_object_from_point_client;

  CallServicesNode() : Node("callservicesnode") {
    this->add_pick_primitives_client =
        this->create_client<frida_interfaces::srv::AddPickPrimitives>(
            ADD_PICK_PRIMITIVES_SERVICE);

    this->remove_plane_client =
        this->create_client<frida_interfaces::srv::RemovePlane>(
            REMOVE_PLANE_SERVICE);

    this->cluster_object_from_point_client =
        this->create_client<frida_interfaces::srv::ClusterObjectFromPoint>(
            CLUSTER_OBJECT_SERVICE);

    RCLCPP_INFO(this->get_logger(), "Clients created, waiting for services");

    this->add_pick_primitives_client->wait_for_service(
        std::chrono::seconds(30));
    RCLCPP_INFO(this->get_logger(), "Service add_pick_primitives ready");
    this->remove_plane_client->wait_for_service(std::chrono::seconds(30));
    RCLCPP_INFO(this->get_logger(), "Service remove_plane ready");
    this->cluster_object_from_point_client->wait_for_service(
        std::chrono::seconds(30));
    RCLCPP_INFO(this->get_logger(), "callservicesnode ready");
  }
};

class TestsNode : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub;
  rclcpp::Service<frida_interfaces::srv::PickPerceptionService>::SharedPtr
      pick_perception_service;

  rclcpp::Service<frida_interfaces::srv::PlacePerceptionService>::SharedPtr
      place_perception_service;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      place_cloud_publisher;
  bool cloud_received = false;
  geometry_msgs::msg::PointStamped::SharedPtr last_point;
  geometry_msgs::msg::PointStamped::SharedPtr point;

  std::shared_ptr<CallServicesNode> call_services_node;

  rclcpp::TimerBase::SharedPtr timer;

public:
  bool testing = false;
  TestsNode(std::shared_ptr<CallServicesNode> call_services_node)
      : Node("tests_node") {

    if (call_services_node == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "CallServicesNode is null");
      throw std::runtime_error("CallServicesNode is null");
    }

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    qos.keep_last(1);

    this->cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        POINT_CLOUD_TOPIC, qos,
        std::bind(&TestsNode::cloud_callback, this, std::placeholders::_1));

    this->place_cloud_publisher =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            PLACE_CLOUD_TOPIC_PUBLISHER, rclcpp::SensorDataQoS());

    this->testing = this->declare_parameter("testing", testing);

    if (testing) {
      RCLCPP_INFO(this->get_logger(), "Testing mode enabled");
      this->point_sub =
          this->create_subscription<geometry_msgs::msg::PointStamped>(
              "/clicked_point", rclcpp::SensorDataQoS(),
              std::bind(&TestsNode::point_callback, this,
                        std::placeholders::_1));
    }

    this->call_services_node = call_services_node;

    RCLCPP_INFO(this->get_logger(), "Clients created");

    this->point = std::make_shared<geometry_msgs::msg::PointStamped>();

    this->last_point = std::make_shared<geometry_msgs::msg::PointStamped>();

    this->pick_perception_service =
        this->create_service<frida_interfaces::srv::PickPerceptionService>(
            PICK_PERCEPTION_SERVICE,
            std::bind(&TestsNode::pick_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));

    this->place_perception_service =
        this->create_service<frida_interfaces::srv::PlacePerceptionService>(
            PLACE_PERCEPTION_SERVICE,
            std::bind(&TestsNode::place_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "Main Service created");
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    cloud_received = true;
  }

  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    this->last_point = msg;
    RCLCPP_INFO(this->get_logger(), "Received point");
  }

  STATUS_RESPONSE
  service_from_point(geometry_msgs::msg::PointStamped point,
                     sensor_msgs::msg::PointCloud2::SharedPtr cloud) {

    auto request = std::make_shared<
        frida_interfaces::srv::ClusterObjectFromPoint::Request>();
    request->point = point;

    RCLCPP_INFO(this->get_logger(), "Sending request");

    auto response = this->call_services_node->cluster_object_from_point_client
                        ->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Waiting for response");

    auto status = wait_for_future_with_timeout<
        frida_interfaces::srv::ClusterObjectFromPoint>(
        response, this->call_services_node->get_node_base_interface(), 500ms);

    RCLCPP_INFO(this->get_logger(), "Response received");

    auto result = response.future.get();

    RCLCPP_INFO(this->get_logger(), "Response: %d", result->status);

    if (result->status == OK) {
      RCLCPP_INFO(this->get_logger(), "Clustered object");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error clustering object with code %d",
                   result->status);
      return result->status;
    }

    *cloud = result->cluster;

    auto req2table =
        std::make_shared<frida_interfaces::srv::RemovePlane::Request>();

    req2table->extract_or_remove = false;
    req2table->close_point.header.frame_id = point.header.frame_id;
    req2table->close_point.header.stamp = point.header.stamp;
    req2table->close_point.point.x = point.point.x;
    req2table->close_point.point.y = point.point.y;
    req2table->close_point.point.z = point.point.z;

    RCLCPP_INFO(this->get_logger(), "Sending request to remove plane");

    auto res2table =
        this->call_services_node->remove_plane_client->async_send_request(
            req2table);

    RCLCPP_INFO(this->get_logger(), "Waiting for response");

    auto status2 =
        wait_for_future_with_timeout<frida_interfaces::srv::RemovePlane>(
            res2table, this->call_services_node->get_node_base_interface(),
            500ms);

    RCLCPP_INFO(this->get_logger(), "Response received");

    auto result2table = res2table.get();

    RCLCPP_INFO(this->get_logger(), "Response: %d",
                result2table->health_response);

    if (result2table->health_response == OK) {
      RCLCPP_INFO(this->get_logger(), "Removed plane");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error removing plane");
      //   return;
    }

    auto req3 =
        std::make_shared<frida_interfaces::srv::AddPickPrimitives::Request>();

    req3->is_plane = true;

    req3->is_object = false;

    req3->cloud = result2table->cloud;

    RCLCPP_INFO(this->get_logger(), "Sending request to add plane");

    auto res3 = this->call_services_node->add_pick_primitives_client
                    ->async_send_request(req3);

    RCLCPP_INFO(this->get_logger(), "Waiting for response");

    auto status3 =
        wait_for_future_with_timeout<frida_interfaces::srv::AddPickPrimitives>(
            res3, this->call_services_node->get_node_base_interface(), 500ms);

    RCLCPP_INFO(this->get_logger(), "Response received");

    auto req4 =
        std::make_shared<frida_interfaces::srv::AddPickPrimitives::Request>();

    req4->is_plane = false;

    req4->is_object = true;

    req4->cloud = result->cluster;

    RCLCPP_INFO(this->get_logger(), "Sending request to add object");

    auto res4 = this->call_services_node->add_pick_primitives_client
                    ->async_send_request(req4);

    RCLCPP_INFO(this->get_logger(), "Waiting for response");

    auto status4 =
        wait_for_future_with_timeout<frida_interfaces::srv::AddPickPrimitives>(
            res4, this->call_services_node->get_node_base_interface(), 500ms);

    RCLCPP_INFO(this->get_logger(), "Response: %d", res4.get()->status);

    auto req5 = std::make_shared<
        frida_interfaces::srv::ClusterObjectFromPoint::Request>();
    req5->point = point;
    req5->is_get_all_other_surrounding_objects = true;
    RCLCPP_INFO(this->get_logger(), "Sending request for other objects");
    auto res5 = this->call_services_node->cluster_object_from_point_client
                    ->async_send_request(req5);
    RCLCPP_INFO(this->get_logger(), "Waiting for response");
    auto status5 = wait_for_future_with_timeout<
        frida_interfaces::srv::ClusterObjectFromPoint>(
        res5, this->call_services_node->get_node_base_interface(), 500ms);
    RCLCPP_INFO(this->get_logger(), "Response received");
    auto result5 = res5.future.get();
    RCLCPP_INFO(this->get_logger(), "Response: %d", result5->status);
    if (result5->status == OK) {
      RCLCPP_INFO(this->get_logger(), "Clustered other objects");
      auto req6 =
          std::make_shared<frida_interfaces::srv::AddPickPrimitives::Request>();
      req6->is_plane = false;
      req6->is_object = true;
      req6->is_other_objects = true;
      req6->cloud = result5->cluster;
      RCLCPP_INFO(this->get_logger(), "Sending request to add other objects");
      auto res6 = this->call_services_node->add_pick_primitives_client
                      ->async_send_request(req6);
      RCLCPP_INFO(this->get_logger(), "Waiting for response");
      auto status6 = wait_for_future_with_timeout<
          frida_interfaces::srv::AddPickPrimitives>(
          res6, this->call_services_node->get_node_base_interface(), 500ms);
      RCLCPP_INFO(this->get_logger(), "Response received");
      RCLCPP_INFO(this->get_logger(), "Response: %d", res6.get()->status);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Error clustering other objects with code %d",
                   result5->status);
    }

    return OK;
  }

  STATUS_RESPONSE
  place_service(const std::shared_ptr<
                    frida_interfaces::srv::PlacePerceptionService::Request>
                    request,
                sensor_msgs::msg::PointCloud2::SharedPtr cloud) {

    auto req_extract_table =
        std::make_shared<frida_interfaces::srv::RemovePlane::Request>();

    req_extract_table->extract_or_remove = false;
    auto place_params = request->place_params;
    req_extract_table->min_height =
        place_params.table_height - place_params.table_height_tolerance;
    req_extract_table->max_height =
        place_params.table_height + place_params.table_height_tolerance;

    RCLCPP_INFO(this->get_logger(), "Sending request to remove plane");

    auto res_extract_table =
        this->call_services_node->remove_plane_client->async_send_request(
            req_extract_table);

    RCLCPP_INFO(this->get_logger(), "Waiting for response");

    auto status2 =
        wait_for_future_with_timeout<frida_interfaces::srv::RemovePlane>(
            res_extract_table,
            this->call_services_node->get_node_base_interface(), 500ms);

    RCLCPP_INFO(this->get_logger(), "Response received");

    auto result_extract_table = res_extract_table.get();

    RCLCPP_INFO(this->get_logger(), "Response: %d",
                result_extract_table->health_response);

    if (result_extract_table->health_response == OK) {
      RCLCPP_INFO(this->get_logger(), "Removed plane");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error removing plane");
      //   return;
    }

    auto req3 =
        std::make_shared<frida_interfaces::srv::AddPickPrimitives::Request>();

    req3->is_plane = true;

    req3->is_object = false;

    req3->cloud = result_extract_table->cloud;

    RCLCPP_INFO(this->get_logger(), "Sending request to add plane");

    auto res3 = this->call_services_node->add_pick_primitives_client
                    ->async_send_request(req3);

    RCLCPP_INFO(this->get_logger(), "Waiting for response");

    auto status3 =
        wait_for_future_with_timeout<frida_interfaces::srv::AddPickPrimitives>(
            res3, this->call_services_node->get_node_base_interface(), 500ms);

    RCLCPP_INFO(this->get_logger(), "Response received");

    // return plane cloud
    *cloud = result_extract_table->cloud;

    return OK;
  }

  void run() {
    // RCLCPP_INFO(this->get_logger(), "Timer callback");
    if (this->point == this->last_point) {
      // RCLCPP_INFO(this->get_logger(), "Same point, skiping processing");
      return;
    }

    this->point = this->last_point;

    if (!cloud_received) {
      return;
    }

    std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud =
        std::make_shared<sensor_msgs::msg::PointCloud2>();

    this->service_from_point(*this->point, cloud);
  }

  void pick_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<
          frida_interfaces::srv::PickPerceptionService::Request>
          request,
      std::shared_ptr<frida_interfaces::srv::PickPerceptionService::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "Pick service callback");
    // response->cluster_result = OK;

    std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud =
        std::make_shared<sensor_msgs::msg::PointCloud2>();

    auto res = this->service_from_point(request->point, cloud);

    response->cluster_result = *cloud;
    // response->status = res;
  }

  void place_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<
          frida_interfaces::srv::PlacePerceptionService::Request>
          request,
      std::shared_ptr<frida_interfaces::srv::PlacePerceptionService::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "Place service callback");

    std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud =
        std::make_shared<sensor_msgs::msg::PointCloud2>();
    auto res = this->place_service(request, cloud);
    RCLCPP_INFO(this->get_logger(), "Returning plane cloud");
    this->place_cloud_publisher->publish(*cloud);
    response->cluster_result = *cloud;
  }

  void loop() {
    while (rclcpp::ok()) {
      run();
      // wait for 1 second
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("tests_main"), "Starting Tests Node");

  rclcpp::executors::MultiThreadedExecutor::SharedPtr multithreaded_executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::shared_ptr<CallServicesNode> call_services_node =
      std::make_shared<CallServicesNode>();

  std::shared_ptr<TestsNode> node =
      std::make_shared<TestsNode>(call_services_node);

  multithreaded_executor->add_node(node);
  if (node->testing) {
    std::thread thread(&TestsNode::loop, node);
    multithreaded_executor->spin();
    if (thread.joinable()) {
      thread.join();
    }
    rclcpp::shutdown();
    return 0;
  } else {
    multithreaded_executor->spin();
  }

  rclcpp::shutdown();

  return 0;
}