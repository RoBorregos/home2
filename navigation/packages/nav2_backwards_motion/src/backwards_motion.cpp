#include "nav2_backwards_motion/backwards_motion.hpp"
#include <string>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_backwards_motion
{

BackwardsMotion::BackwardsMotion(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  config.blackboard->get("node", node_);
  if (!node_) {
    throw std::runtime_error("Failed to get node from blackboard");
  }
  
  // Create publisher for velocity commands
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  
  // Get default parameters - using try/catch since the parameters might already be declared
  try {
    node_->declare_parameter("backwards_speed", -0.2);  // Default speed
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    // Parameter already declared, nothing to do
  }
  
  try {
    node_->declare_parameter("backwards_time_seconds", 3.0);  // Default duration
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    // Parameter already declared, nothing to do
  }
  
  backwards_speed_ = node_->get_parameter("backwards_speed").as_double();
  time_seconds_ = node_->get_parameter("backwards_time_seconds").as_double();
}

// Define required BT ports
BT::PortsList BackwardsMotion::providedPorts()
{
  return {
    BT::InputPort<double>("speed", -0.2, "Backwards speed in m/s (negative value)"),
    BT::InputPort<double>("time_seconds", 3.0, "How long to move backwards in seconds")
  };
}

BT::NodeStatus BackwardsMotion::tick()
{
  // Get input parameters from the BT, or use default values
  double speed = backwards_speed_;
  double duration = time_seconds_;
  
  getInput("speed", speed);
  getInput("time_seconds", duration);
  
  return executeBackwardsMotion(speed, duration);
}

BT::NodeStatus BackwardsMotion::executeBackwardsMotion(double speed, double duration)
{
  RCLCPP_INFO(node_->get_logger(), 
    "Starting backwards motion at %.2f m/s for %.1f seconds", speed, duration);
  
  // Start backwards motion
  auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
  twist_msg->linear.x = speed;  // Negative value for backwards
  vel_pub_->publish(std::move(twist_msg));
  
  // Wait for the specified duration
  auto start_time = node_->now();
  while ((node_->now() - start_time).seconds() < duration) {
    // Check if we need to preempt
    if (!rclcpp::ok()) {
      break;
    }
    
    // Keep publishing commands to ensure continuous motion
    twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = speed;
    vel_pub_->publish(std::move(twist_msg));
    
    // Small delay to avoid flooding the topic
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // Stop motion
  twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
  twist_msg->linear.x = 0.0;
  vel_pub_->publish(std::move(twist_msg));
  
  RCLCPP_INFO(node_->get_logger(), "Completed backwards motion");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_backwards_motion

// Register the BT node with pluginlib
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_backwards_motion::BackwardsMotion>("BackwardsMotion");
}