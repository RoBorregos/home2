#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "frida_interfaces/srv/map_areas.hpp"
#include <memory>
#include <fstream>
#include <nlohmann/json.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AreasServices : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit AreasServices(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("areas_wrapper", options)
  {
    this->declare_parameter("map_name", "default_map");
    this->declare_parameter("autostart", false);
    RCLCPP_INFO(this->get_logger(), "AreasWrapper Node created, waiting for configuration...");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring AreasWrapper Node ...");
    service_ = this->create_service<frida_interfaces::srv::MapAreas>(
      "areas_json",
      std::bind(&AreasServices::areas_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Activating AreasWrapper Node ...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating AreasWrapper Node ...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up AreasWrapper Node ...");
    service_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down AreasWrapper Node from state %s...", state.label().c_str());
    service_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void areas_callback(
    const std::shared_ptr<frida_interfaces::srv::MapAreas::Request> request,
    std::shared_ptr<frida_interfaces::srv::MapAreas::Response> response)
  {
    (void)request; // Empty Request
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_WARN(this->get_logger(), "Service called but node is not ACTIVE");
      return;
    }

    try {
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("map_context");
      std::string map_areas = this->get_parameter("map_name").as_string();
      std::string file_path = package_share_directory + "/maps/areas/areas_" + map_areas + ".json";
      
      std::ifstream f(file_path);
      if (!f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
        return;
      }
      nlohmann::json data = nlohmann::json::parse(f);
      response->areas = data.dump();
      RCLCPP_INFO(this->get_logger(), "Map Areas Sent");
    }
    catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error sending map areas: %s", e.what());
    }
    catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown error sending map areas!");
    }
  }

  rclcpp::Service<frida_interfaces::srv::MapAreas>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AreasServices>(rclcpp::NodeOptions());
  
  /*bool autostart = node->get_parameter("autostart").as_bool();
  if (autostart) {
    node->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
    node->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  }*/

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
