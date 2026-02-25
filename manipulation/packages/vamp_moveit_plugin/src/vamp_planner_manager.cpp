#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include "vamp_moveit_plugin/vamp_planning_context.h"
#include <pluginlib/class_list_macros.hpp>

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
    (void)model;
    (void)parameter_namespace;
    node_ = node;
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
};
}

PLUGINLIB_EXPORT_CLASS(vamp_moveit_plugin::VampPlannerManager, planning_interface::PlannerManager)