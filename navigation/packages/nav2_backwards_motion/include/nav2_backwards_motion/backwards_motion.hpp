#ifndef NAV2_BACKWARDS_MOTION__BACKWARDS_MOTION_HPP_
#define NAV2_BACKWARDS_MOTION__BACKWARDS_MOTION_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_backwards_motion
{

class BackwardsMotion : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the BackwardsMotion BT node
   * @param name Name of the BT node
   * @param config BT node configuration
   */
  BackwardsMotion(const std::string& name, const BT::NodeConfiguration& config);

  /**
   * @brief Defines the ports (inputs/outputs) for the BT node
   * @return PortsList containing input and output ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Main execution function called by the behavior tree
   * @return NodeStatus (SUCCESS, FAILURE, or RUNNING)
   */
  BT::NodeStatus tick() override;

private:
  // ROS node
  rclcpp::Node::SharedPtr node_;
  
  // Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  
  // Parameters
  double backwards_speed_;  // Speed for backwards motion (negative value)
  double time_seconds_;     // Time to move backwards in seconds
  
  /**
   * @brief Executes the backwards motion for specified duration
   * @return NodeStatus SUCCESS if completed, FAILURE otherwise
   */
  BT::NodeStatus executeBackwardsMotion(double speed, double duration);
};

}  // namespace nav2_backwards_motion

#endif  // NAV2_BACKWARDS_MOTION__BACKWARDS_MOTION_HPP_