#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <algorithm>
#include <cmath>

namespace nav_main
{

class ControllerDualshock : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit ControllerDualshock(const rclcpp::NodeOptions & options)
  : Node("controller_dualshock", options),
    rotation_speed_(0.1),
    linear_speed_(0.2),
    deadzone_(0.4)
  {
    // Declare parameters so they can be overridden from launch
    this->declare_parameter("rotation_speed", rotation_speed_);
    this->declare_parameter("linear_speed", linear_speed_);
    this->declare_parameter("deadzone", deadzone_);

    this->get_parameter("rotation_speed", rotation_speed_);
    this->get_parameter("linear_speed", linear_speed_);
    this->get_parameter("deadzone", deadzone_);

    RCLCPP_INFO(this->get_logger(),
      "Starting ControllerDualshock — linear: %.2f, rotation: %.2f, deadzone: %.2f",
      linear_speed_, rotation_speed_, deadzone_);

    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&ControllerDualshock::joy_callback, this, std::placeholders::_1));

    // Nav2 action client for cancelling goals
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/navigate_to_pose");
  }

private:
  double apply_deadzone(double value) const
  {
    if (std::abs(value) < deadzone_) {
      return 0.0;
    }
    return value;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Safety: make sure axes/buttons arrays are large enough
    if (msg->axes.size() < 8 || msg->buttons.size() < 4) {
      return;
    }

    // --- Velocity command ---
    geometry_msgs::msg::Twist twist;
    twist.linear.x = msg->axes[1] * linear_speed_;
    twist.angular.z = apply_deadzone(msg->axes[0]) * rotation_speed_;
    cmd_vel_pub_->publish(twist);

    // --- Square button (buttons[3]): cancel navigation goal ---
    if (msg->buttons[3] == 1) {
      nav2_client_->async_cancel_all_goals();
      RCLCPP_INFO(this->get_logger(), "Navigation goal cancelled.");
    }

    // --- D-pad left (axes[6] > 0.5): increase linear speed ---
    if (msg->axes[6] > 0.5) {
      linear_speed_ = std::min(linear_speed_ + 0.05, 1.0);
    }

    // --- D-pad right (axes[6] < -0.5): decrease linear speed ---
    if (msg->axes[6] < -0.5) {
      linear_speed_ = std::max(linear_speed_ - 0.05, 0.05);
    }

    // --- D-pad up (axes[7] > 0.5): increase rotation speed ---
    if (msg->axes[7] > 0.5) {
      rotation_speed_ = std::min(rotation_speed_ + 0.05, 1.0);
    }

    // --- D-pad down (axes[7] < -0.5): decrease rotation speed ---
    if (msg->axes[7] < -0.5) {
      rotation_speed_ = std::max(rotation_speed_ - 0.05, 0.05);
    }
  }

  // Members
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;

  double rotation_speed_;
  double linear_speed_;
  double deadzone_;
};

}  // namespace nav_main

RCLCPP_COMPONENTS_REGISTER_NODE(nav_main::ControllerDualshock)
