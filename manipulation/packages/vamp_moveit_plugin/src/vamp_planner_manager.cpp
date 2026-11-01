#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_interface/planning_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vamp_moveit_plugin/vamp_client.hpp"
#include "vamp_moveit_plugin/vamp_planning_context.hpp"

namespace vamp_moveit_plugin
{
/// MoveIt planner plugin for VAMP; it owns the server client and the OMPL planner used as fallback.
class VampPlannerManager : public planning_interface::PlannerManager
{
public:
  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override
  {
    logger_ = node->get_logger().get_child("vamp");
    const std::string prefix = parameter_namespace + ".";
    node->get_parameter_or(prefix + "service_name", params_.service_name, params_.service_name);
    node->get_parameter_or(prefix + "end_effector_link", params_.end_effector_link, params_.end_effector_link);
    node->get_parameter_or(prefix + "request_timeout", params_.request_timeout, params_.request_timeout);
    node->get_parameter_or(prefix + "fallback_attempts", params_.fallback_attempts, params_.fallback_attempts);

    client_ = std::make_shared<VampClient>(params_.service_name);

    // OMPL reads its planner configs from this same namespace, so the pipeline must carry the OMPL settings too
    try
    {
      fallback_loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
          "moveit_core", "planning_interface::PlannerManager");
      fallback_ = fallback_loader_->createSharedInstance("ompl_interface/OMPLPlanner");
      if (!fallback_->initialize(model, node, parameter_namespace))
      {
        RCLCPP_ERROR(logger_, "OMPL fallback failed to initialize; VAMP failures will not be recovered");
        fallback_.reset();
      }
    }
    catch (const pluginlib::PluginlibException& e)
    {
      RCLCPP_ERROR(logger_, "OMPL fallback could not be loaded (%s); VAMP failures will not be recovered", e.what());
      fallback_.reset();
    }

    RCLCPP_INFO(logger_, "VAMP planner ready (server '%s', OMPL fallback %s)", params_.service_name.c_str(),
                fallback_ ? "enabled" : "disabled");
    return true;
  }

  std::string getDescription() const override
  {
    return "VAMP";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs = { "vamp" };
  }

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    if (!canServiceRequest(req))
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return nullptr;
    }
    auto context = std::make_shared<VampPlanningContext>("vamp", req.group_name, params_, client_, fallback_, logger_);
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return context;
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override
  {
    return !req.group_name.empty() && req.trajectory_constraints.constraints.empty();
  }

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("vamp");
  VampPlannerParams params_;
  std::shared_ptr<const VampClient> client_;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> fallback_loader_;
  std::shared_ptr<planning_interface::PlannerManager> fallback_;
};

}  // namespace vamp_moveit_plugin

PLUGINLIB_EXPORT_CLASS(vamp_moveit_plugin::VampPlannerManager, planning_interface::PlannerManager)
