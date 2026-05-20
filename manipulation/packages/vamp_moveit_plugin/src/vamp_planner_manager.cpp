#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include "vamp_moveit_plugin/vamp_planning_context.h"
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>

namespace vamp_moveit_plugin
{
class VampPlannerManager : public planning_interface::PlannerManager
{
public:
  VampPlannerManager() {}
  ~VampPlannerManager() override {}

  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override
  {
    node_ = node;
    robot_model_ = model;
    parameter_namespace_ = parameter_namespace;

    // Load OMPL as fallback so VAMP plans rejected by FCL post-validation
    // get retried automatically — caller gets one consistent interface even
    // when VAMP's sphere model misses something. If OMPL isn't installed or
    // initialization fails, we still work as a pure VAMP planner (just
    // without recovery for false negatives), so failures here are warnings,
    // not fatal.
    try {
      ompl_loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
          "moveit_core", "planning_interface::PlannerManager");
      ompl_manager_ = ompl_loader_->createSharedInstance("ompl_interface/OMPLPlanner");
      if (!ompl_manager_->initialize(model, node, parameter_namespace)) {
        RCLCPP_WARN(node_->get_logger(),
          "OMPL fallback installed but initialize() returned false; disabling.");
        ompl_manager_.reset();
      } else {
        RCLCPP_INFO(node_->get_logger(),
          "OMPL fallback ready (auto-retries when VAMP plan fails FCL).");
      }
    } catch (const pluginlib::PluginlibException& e) {
      RCLCPP_WARN(node_->get_logger(),
        "OMPL fallback not available (%s); running VAMP-only.", e.what());
      ompl_manager_.reset();
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(),
        "OMPL fallback init threw (%s); running VAMP-only.", e.what());
      ompl_manager_.reset();
    }
    return true;
  }

  std::string getDescription() const override { return "VAMP"; }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("vamp");
  }

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    if (req.group_name.empty()) {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return nullptr;
    }

    auto context = std::make_shared<VampPlanningContext>(
        "vamp_context", req.group_name, planning_scene->getRobotModel(), node_);

    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    context->setOmplFallback(ompl_manager_);

    return context;
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override
  {
    (void)pcs;
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::string parameter_namespace_;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> ompl_loader_;
  std::shared_ptr<planning_interface::PlannerManager> ompl_manager_;
};
}

PLUGINLIB_EXPORT_CLASS(vamp_moveit_plugin::VampPlannerManager, planning_interface::PlannerManager)
