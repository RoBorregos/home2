#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <variant>

#include <frida_interfaces/srv/add_pick_primitives.hpp>
#include <frida_interfaces/srv/cluster_object_from_point.hpp>
#include <frida_interfaces/srv/remove_plane.hpp>

#include <frida_constants/manip_3d.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <perception_3d/macros.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class TestsNode : public rclcpp::Node {
private:
  rclcpp::Client<frida_interfaces::srv::AddPickPrimitives>::SharedPtr
      add_pick_primitives_client;
  rclcpp::Client<frida_interfaces::srv::RemovePlane>::SharedPtr
      remove_plane_client;
  rclcpp::Client<frida_interfaces::srv::ClusterObjectFromPoint>::SharedPtr
      cluster_object_from_point_client;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub;

  bool cloud_received = false;
  geometry_msgs::msg::PointStamped::SharedPtr last_point;
  geometry_msgs::msg::PointStamped::SharedPtr point;

  rclcpp::TimerBase::SharedPtr timer;

public:
  TestsNode() : Node("tests_node") {
    this->cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        POINT_CLOUD_TOPIC, rclcpp::SensorDataQoS(),
        std::bind(&TestsNode::cloud_callback, this, std::placeholders::_1));

    this->point_sub =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", rclcpp::SensorDataQoS(),
            std::bind(&TestsNode::point_callback, this, std::placeholders::_1));

    this->add_pick_primitives_client =
        this->create_client<frida_interfaces::srv::AddPickPrimitives>(
            ADD_PICK_PRIMITIVES_SERVICE);

    this->remove_plane_client =
        this->create_client<frida_interfaces::srv::RemovePlane>(
            REMOVE_PLANE_SERVICE);

    this->cluster_object_from_point_client =
        this->create_client<frida_interfaces::srv::ClusterObjectFromPoint>(
            CLUSTER_OBJECT_SERVICE);

    RCLCPP_INFO(this->get_logger(), "Clients created");

    this->point = std::make_shared<geometry_msgs::msg::PointStamped>();

    this->last_point = std::make_shared<geometry_msgs::msg::PointStamped>();

    // this->timer = this->create_wall_timer(
    //     500ms, std::bind(&TestsNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Timer created");

    // wait for first point cloud
    // while(!this->cloud_sub->return_message().get()) {
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received point cloud");
    cloud_received = true;
  }

  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    this->last_point = msg;
    RCLCPP_INFO(this->get_logger(), "Received point");
  }

  void run() {
    // RCLCPP_INFO(this->get_logger(), "Timer callback");
    if (point == last_point) {
      RCLCPP_INFO(this->get_logger(), "Same point, skiping processing");
      return;
    }

    point = last_point;

    if (!cloud_received) {
      // RCLCPP_INFO(this->get_logger(), "No point cloud received yet");
      return;
    }

    auto request = std::make_shared<
        frida_interfaces::srv::ClusterObjectFromPoint::Request>();
    request->point = *point;

    RCLCPP_INFO(this->get_logger(), "Sending request");

    // auto res = this->cluster_object_from_point_client->

    auto response =
        this->cluster_object_from_point_client->async_send_request(request);

    auto start_time = this->now();

    while (rclcpp::ok()) {
      auto status = response.wait_for(std::chrono::milliseconds(100));
      // rclcpp::executors::spin_node_until_future_complete(this, response);

      if (status == std::future_status::ready) {
        break;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Waiting for response, current time waited: %f",
                  (this->now() - start_time).seconds());

      // Check for timeout (e.g., 5 seconds)
      if ((this->now() - start_time).seconds() > 60.0) {
        RCLCPP_ERROR(this->get_logger(), "Service call timed out");
        return;
      }
    }

    // auto response =
    //     this->cluster_object_from_point_client->async_send_request(request);

    // RCLCPP_INFO(this->get_logger(), "Waiting for response");

    // // if
    // (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
    // //                                        response) !=
    // //     rclcpp::FutureReturnCode::SUCCESS) {
    // //   RCLCPP_ERROR(this->get_logger(), "Error waiting for response");
    // //   return;
    // // }

    // auto start_time = this->now();
    // while (rclcpp::ok()) {
    //   auto status = response.future.wait_for(std::chrono::milliseconds(100));
    //   if (status == std::future_status::ready) {
    //     break;
    //   }
    //   RCLCPP_INFO(this->get_logger(),
    //               "Waiting for response, current time waited: %f",
    //               (this->now() - start_time).seconds());
    //   RCLCPP_INFO(this->get_logger(), "Waiting for response status %d",
    //   status);
    //   // if (response.get()->status == OK) {
    //   //   RCLCPP_INFO(this->get_logger(), "Clustered object AAAAAA");
    //   //   break;
    //   // }
    //   // RCLCPP_INFO(this->get_logger(), "Waiting for response status %d

    //   // Check for timeout (e.g., 5 seconds)
    //   if ((this->now() - start_time).seconds() > 60.0) {
    //     RCLCPP_ERROR(this->get_logger(), "Service call timed out");
    //     return;
    //   }
    // }

    RCLCPP_INFO(this->get_logger(), "Response received");

    auto result = response.future.get();

    RCLCPP_INFO(this->get_logger(), "Response: %d", result->status);

    if (result->status == OK) {
      RCLCPP_INFO(this->get_logger(), "Clustered object");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error clustering object with code %d",
                   result->status);
      return;
    }

    auto req2table =
        std::make_shared<frida_interfaces::srv::RemovePlane::Request>();

    req2table->extract_or_remove = false;
    req2table->close_point.header.frame_id = point->header.frame_id;
    req2table->close_point.header.stamp = point->header.stamp;
    req2table->close_point.point.x = point->point.x;
    req2table->close_point.point.y = point->point.y;
    req2table->close_point.point.z = point->point.z;

    RCLCPP_INFO(this->get_logger(), "Sending request to remove plane");

    auto res2table = this->remove_plane_client->async_send_request(req2table);

    RCLCPP_INFO(this->get_logger(), "Waiting for response");

    start_time = this->now();

    while (rclcpp::ok()) {
      auto status = res2table.wait_for(std::chrono::milliseconds(100));
      if (status == std::future_status::ready) {
        break;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for response status %d, time %f",
                  status, (this->now() - start_time).seconds());

      // Check for timeout (e.g., 5 seconds)
      if ((this->now() - start_time).seconds() > 60.0) {
        RCLCPP_ERROR(this->get_logger(), "Service call timed out");
        return;
      }
    }

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

    auto res3 = this->add_pick_primitives_client->async_send_request(req3);

    RCLCPP_INFO(this->get_logger(), "Waiting for response");

    start_time = this->now();

    while (rclcpp::ok()) {
      auto status = res3.wait_for(std::chrono::milliseconds(100));
      if (status == std::future_status::ready) {
        break;
      }

      RCLCPP_INFO(this->get_logger(), "Waiting for response status %d, time %f",
                  status, (this->now() - start_time).seconds());

      // Check for timeout (e.g., 5 seconds)
      if ((this->now() - start_time).seconds() > 60.0) {
        RCLCPP_ERROR(this->get_logger(), "Service call timed out");
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Response received");

    auto req4 =
        std::make_shared<frida_interfaces::srv::AddPickPrimitives::Request>();

    req4->is_plane = false;

    req4->is_object = true;

    req4->cloud = result->cluster;

    RCLCPP_INFO(this->get_logger(), "Sending request to add object");

    auto res4 = this->add_pick_primitives_client->async_send_request(req4);

    RCLCPP_INFO(this->get_logger(), "Waiting for response");

    start_time = this->now();

    while (rclcpp::ok()) {
      auto status = res4.wait_for(std::chrono::milliseconds(100));
      if (status == std::future_status::ready) {
        break;
      }

      RCLCPP_INFO(this->get_logger(), "Waiting for response status %d, time %f",
                  status, (this->now() - start_time).seconds());

      // Check for timeout (e.g., 5 seconds)
      if ((this->now() - start_time).seconds() > 60.0) {
        RCLCPP_ERROR(this->get_logger(), "Service call timed out");
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Response: %d", res4.get()->status);

    return;
  }

  void loop() {
    while (rclcpp::ok()) {
      run();
      // wait for 1 second
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  // void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received point");
  //   auto request = std::make_shared<
  //       frida_interfaces::srv::ClusterObjectFromPoint::Request>();
  //   request->point = *msg;

  //   RCLCPP_INFO(this->get_logger(), "Sending request");

  //   auto response =
  //       this->cluster_object_from_point_client->async_send_request(request);

  //   RCLCPP_INFO(this->get_logger(), "Waiting for response");
  //   try {

  //     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
  //                                            response) !=
  //         rclcpp::FutureReturnCode::SUCCESS) {
  //       RCLCPP_ERROR(this->get_logger(), "Error waitin");
  //       return;
  //     }

  //     RCLCPP_INFO(this->get_logger(), "Response received");

  //     auto result = response.get();

  //     RCLCPP_INFO(this->get_logger(), "Response: %d", result->status);

  //     if (result->status == OK) {
  //       RCLCPP_INFO(this->get_logger(), "Clustered object");
  //     } else {
  //       RCLCPP_ERROR(this->get_logger(), "Error clustering object with code
  //       %d",
  //                    result->status);
  //       return;
  //     }

  //     auto req2table =
  //         std::make_shared<frida_interfaces::srv::RemovePlane::Request>();

  //     req2table->extract_or_remove = false;

  //     RCLCPP_INFO(this->get_logger(), "Sending request to remove plane");

  //     auto res2table =
  //     this->remove_plane_client->async_send_request(req2table);

  //     RCLCPP_INFO(this->get_logger(), "Waiting for response");

  //     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
  //                                            res2table) !=
  //         rclcpp::FutureReturnCode::SUCCESS) {
  //       RCLCPP_ERROR(this->get_logger(), "Error removing plane");
  //       return;
  //     }

  //     RCLCPP_INFO(this->get_logger(), "Response received");

  //     auto result2table = res2table.get();

  //     RCLCPP_INFO(this->get_logger(), "Response: %d",
  //                 result2table->health_response);

  //     if (result2table->health_response == OK) {
  //       RCLCPP_INFO(this->get_logger(), "Removed plane");
  //     } else {
  //       RCLCPP_ERROR(this->get_logger(), "Error removing plane");
  //       //   return;
  //     }

  //     auto req3 =
  //         std::make_shared<frida_interfaces::srv::AddPickPrimitives::Request>();

  //     req3->is_plane = true;
  //     req3->is_object = false;
  //     req3->cloud = result2table->cloud;

  //     RCLCPP_INFO(this->get_logger(), "Sending request to add plane");

  //     auto res3 = this->add_pick_primitives_client->async_send_request(req3);

  //     RCLCPP_INFO(this->get_logger(), "Waiting for response");

  //     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
  //                                            res3) !=
  //         rclcpp::FutureReturnCode::SUCCESS) {
  //       RCLCPP_ERROR(this->get_logger(), "Error adding plane");
  //       return;
  //     }

  //     RCLCPP_INFO(this->get_logger(), "Response received");

  //     auto req4 =
  //         std::make_shared<frida_interfaces::srv::AddPickPrimitives::Request>();

  //     req4->is_plane = false;
  //     req4->is_object = true;

  //     req4->cloud = result->cluster;

  //     RCLCPP_INFO(this->get_logger(), "Sending request to add object");

  //     auto res4 = this->add_pick_primitives_client->async_send_request(req4);

  //     RCLCPP_INFO(this->get_logger(), "Waiting for response");

  //     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
  //                                            res4) !=
  //         rclcpp::FutureReturnCode::SUCCESS) {
  //       RCLCPP_ERROR(this->get_logger(), "Error adding object");
  //       return;
  //     }

  //     RCLCPP_INFO(this->get_logger(), "Response: %d", res4.get()->status);
  //   } catch (const std::exception &e) {
  //     RCLCPP_ERROR(this->get_logger(), "Error %s ", e.what());
  //     return;
  //   }
  // }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("tests_main"), "Starting Tests Node");

  rclcpp::executors::MultiThreadedExecutor::SharedPtr multithreaded_executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::shared_ptr<TestsNode> node = std::make_shared<TestsNode>();

  multithreaded_executor->add_node(node);

  std::thread thread(&TestsNode::loop, node);

  multithreaded_executor->spin();

  if (thread.joinable()) {
    thread.join();
  }

  rclcpp::shutdown();

  return 0;
}