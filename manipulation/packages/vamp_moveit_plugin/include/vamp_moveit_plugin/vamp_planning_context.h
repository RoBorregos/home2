#ifndef VAMP_MOVEIT_PLUGIN_VAMP_PLANNING_CONTEXT_H
#define VAMP_MOVEIT_PLUGIN_VAMP_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

namespace vamp_moveit_plugin
{

class VampPlanningContext : public planning_interface::PlanningContext
{
public:
  VampPlanningContext(const std::string& name,
                      const std::string& group,
                      const moveit::core::RobotModelConstPtr& robot_model,
                      const rclcpp::Node::SharedPtr& node);

  ~VampPlanningContext() override;

  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
  bool terminate() override;
  void clear() override;

private:
  bool extractStartState(std::vector<double>& start_state) const;
  bool extractGoalState(std::vector<double>& goal_state) const;

  bool convertPlanningSceneToVamp(void* vamp_env_ptr) const;
  bool convertShapeToVamp(const shapes::Shape* shape,
                          const Eigen::Isometry3d& pose,
                          void* vamp_env_ptr) const;

  bool callVampPlanner(const std::vector<double>& start_state,
                       const std::vector<double>& goal_state,
                       void* vamp_env_ptr,
                       std::vector<std::vector<double>>& waypoints) const;

  robot_trajectory::RobotTrajectoryPtr createTrajectory(
      const std::vector<std::vector<double>>& waypoints) const;
      
  void timeParameterizeTrajectory(
      robot_trajectory::RobotTrajectoryPtr& trajectory) const;

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
  bool terminate_ = false;

  static constexpr size_t FRIDA_DOF = 8;
};

}

#endif