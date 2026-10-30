#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "vamp_moveit_plugin/srv/vamp_plan.hpp"

namespace vamp_moveit_plugin
{
/// Client of the VAMP planning service, spun on its own executor so move_group's executor is never blocked.
class VampClient
{
public:
  using Request = srv::VampPlan::Request;
  using Response = srv::VampPlan::Response;

  explicit VampClient(const std::string& service_name);
  ~VampClient();

  VampClient(const VampClient&) = delete;
  VampClient& operator=(const VampClient&) = delete;

  /// Whether the VAMP server is currently reachable.
  bool isReady() const;

  /// Sends a request and waits for the reply; returns nullopt on timeout.
  std::optional<Response> plan(const Request& request, std::chrono::duration<double> timeout) const;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<srv::VampPlan>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

}  // namespace vamp_moveit_plugin
