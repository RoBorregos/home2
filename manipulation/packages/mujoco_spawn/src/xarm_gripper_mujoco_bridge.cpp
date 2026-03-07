#include "rclcpp/rclcpp.hpp"
#include "xarm_msgs/srv/set_digital_io.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include <memory>
#include <string>

///Node that is in charge of transforming the topic /xarm/set_tgpio_digital - /xarm/set_cgpio_digital into movement in mujoco by ros2_control


class Xarm_gripper_mujoco_bridge : public rclcpp::Node
{
public:
  explicit Xarm_gripper_mujoco_bridge(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node("xarm_gripper_mujoco_bridge", options)
  {
    // Declare parameters
    this->declare_parameter<double>("open_position", 0.056);       
    this->declare_parameter<double>("close_position", 0.0);
    this->declare_parameter<double>("movement_duration", 0.5);   // seconds
    this->declare_parameter<std::string>("joint_name", "rightfinger");
    this->declare_parameter<std::string>("trajectory_controller_name", "joint_trajectory");
    this->declare_parameter<std::string>("controller_manager_ns", "/xarm_gripper_traj_controller");

    // Get parameter
    open_position_ = this->get_parameter("open_position").as_double();
    close_position_ = this->get_parameter("close_position").as_double();
    movement_duration_ = this->get_parameter("movement_duration").as_double();
    joint_name_ = this->get_parameter("joint_name").as_string();
    trajectory_controller_name_ = this->get_parameter("trajectory_controller_name").as_string();
    controller_manager_ns_ = this->get_parameter("controller_manager_ns").as_string();

    setup_trajectory_control();

    // Create Digital IO service servers for both of the binary options
    tgpio_service_ = this->create_service<xarm_msgs::srv::SetDigitalIO>(
      "/xarm/set_tgpio_digital",
      std::bind(&Xarm_gripper_mujoco_bridge::handle_set_tgpio_digital, this,
                std::placeholders::_1, std::placeholders::_2));

    cgpio_service_ = this->create_service<xarm_msgs::srv::SetDigitalIO>(
      "/xarm/set_cgpio_digital",
      std::bind(&Xarm_gripper_mujoco_bridge::handle_set_cgpio_digital, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), 
      "Gripper Bridge initialized\n"
      "  Joint Name: %s\n"
      "  Movement Duration: %.2f sec\n",
      joint_name_.c_str(),
      movement_duration_
    );
  }

private:
  // Communication
  rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr tgpio_service_;
  rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr cgpio_service_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

  // Configuration
  int controlled_joint_;
  double open_position_;
  double close_position_;
  double movement_duration_;
  std::string joint_name_;
  std::string trajectory_controller_name_;
  std::string controller_manager_ns_;
  
  void setup_trajectory_control()
  {
    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      controller_manager_ns_ + "/" + trajectory_controller_name_ ,
      rclcpp::QoS(10));


    RCLCPP_INFO(this->get_logger(), 
      "Using TRAJECTORY control on topic: %s/%s",
      controller_manager_ns_.c_str(),
      trajectory_controller_name_.c_str());
  }

  void handle_set_tgpio_digital(
    const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> request,
    std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
      "[TGPIO] IO: %d, Value: %d (0=open, 1=close), Delay: %.2fs",
      request->ionum, request->value, request->delay_sec);

    if (request->ionum < 1 || request->ionum > 8) {
      response->ret = -1;
      response->message = "Invalid TGPIO IO number (valid range: 1-8)";
      return;
    }

    bool success = execute_gripper_movement(request->value, request->delay_sec);

    if (success) {
      response->ret = 0;
      response->message = request->value == 0 ? "Gripper opened" : "Gripper closed";
    } else {
      response->ret = -1;
      response->message = "Failed to execute gripper movement";
    }
  }

  void handle_set_cgpio_digital(
    const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> request,
    std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
      "[CGPIO] IO: %d, Value: %d (0=open, 1=close), Delay: %.2fs",
      request->ionum, request->value, request->delay_sec);

    if (request->ionum < 1 || request->ionum > 16) {
      response->ret = -1;
      response->message = "Invalid CGPIO IO number (valid range: 1-16)";
      return;
    }

    bool success = execute_gripper_movement(request->value, request->delay_sec);

    if (success) {
      response->ret = 0;
      response->message = request->value == 0 ? "Gripper opened" : "Gripper closed";
    } else {
      response->ret = -1;
      response->message = "Failed to execute gripper movement";
    }
  }

  bool execute_gripper_movement(int direction, float delay_sec)
  {
    try {
      double target_position = (direction == 0) ? open_position_ : close_position_;
      std::string state = (direction == 0) ? "OPEN" : "CLOSE";
      
      RCLCPP_INFO(this->get_logger(),
        "Moving gripper '%s' to %s (%.4f) - Duration: %.2f sec",
        joint_name_.c_str(),
        state.c_str(),
        target_position,
        delay_sec);

      auto trajectory_msg = create_joint_trajectory(target_position, delay_sec);
      trajectory_publisher_->publish(trajectory_msg);

      RCLCPP_DEBUG(this->get_logger(),
        "Trajectory published - Duration: %.2f sec", delay_sec);
      return true;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in gripper movement: %s", e.what());
      return false;
    }
  }

 

  trajectory_msgs::msg::JointTrajectory create_joint_trajectory(
    double target_position, 
    double duration_sec)
  {
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    
    trajectory_msg.header.stamp = this->now();
    trajectory_msg.joint_names.push_back(joint_name_);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(target_position);
    point.velocities.push_back(0.0);
    point.accelerations.push_back(0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

    trajectory_msg.points.push_back(point);

    return trajectory_msg;
  }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Xarm_gripper_mujoco_bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}