#include "vamp_moveit_plugin/vamp_client.hpp"

namespace vamp_moveit_plugin
{
VampClient::VampClient(const std::string& service_name)
{
  node_ = std::make_shared<rclcpp::Node>("vamp_planner_client");
  client_ = node_->create_client<srv::VampPlan>(service_name);
  executor_.add_node(node_);
  spin_thread_ = std::thread([this] { executor_.spin(); });
}

VampClient::~VampClient()
{
  executor_.cancel();
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
}

bool VampClient::isReady() const
{
  return client_->service_is_ready();
}

std::optional<VampClient::Response> VampClient::plan(const Request& request,
                                                     std::chrono::duration<double> timeout) const
{
  auto pending = client_->async_send_request(std::make_shared<Request>(request));
  if (pending.wait_for(timeout) != std::future_status::ready)
  {
    client_->remove_pending_request(pending);
    return std::nullopt;
  }
  return *pending.get();
}

}  // namespace vamp_moveit_plugin
