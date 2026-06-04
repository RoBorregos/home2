#ifndef VAMP_MOVEIT_PLUGIN_VAMP_PLANNING_CONTEXT_H
#define VAMP_MOVEIT_PLUGIN_VAMP_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rclcpp/rclcpp.hpp>
#include "vamp_moveit_plugin/srv/vamp_plan.hpp"
#include <thread>
#include <atomic>

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

  /// Install an OMPL planner manager as fallback. When a VAMP-generated path
  /// fails post-validation against the PlanningScene (sphere model missed
  /// something FCL catches), solve() retries with this manager and returns
  /// OMPL's plan if it succeeds. Pass nullptr to disable fallback.
  void setOmplFallback(std::shared_ptr<planning_interface::PlannerManager> mgr)
  { ompl_manager_ = std::move(mgr); }

private:
  void extractCollisionScene(
      const planning_scene::PlanningSceneConstPtr& scene,
      std::shared_ptr<vamp_moveit_plugin::srv::VampPlan::Request>& request) const;

  bool convertPlanningSceneToVamp(void* vamp_env_ptr) const;
  bool convertShapeToVamp(const shapes::Shape* shape,
                          const Eigen::Isometry3d& pose,
                          void* vamp_env_ptr) const;
  bool callVampPlanner(const std::vector<double>& start_state,
                       const std::vector<double>& goal_state,
                       void* vamp_env_ptr,
                       std::vector<std::vector<double>>& waypoints) const;

  /// Parent node (owned by move_group's executor — DO NOT spin this)
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
  std::atomic<bool> terminate_{false};

  rclcpp::Node::SharedPtr vamp_node_;
  rclcpp::Client<vamp_moveit_plugin::srv::VampPlan>::SharedPtr vamp_client_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> vamp_executor_;
  std::thread vamp_executor_thread_;

  /// Set by VampPlannerManager at construction; used by solve() when VAMP's
  /// path is rejected by FCL post-validation.
  std::shared_ptr<planning_interface::PlannerManager> ompl_manager_;

  /// True if the VAMP-produced trajectory survives a per-waypoint check
  /// against the current PlanningScene. Logs why if not.
  bool validateTrajectoryAgainstScene(
      const robot_trajectory::RobotTrajectory& traj,
      const planning_scene::PlanningSceneConstPtr& scene) const;

  /// Replan with the OMPL fallback (if installed). Returns true and fills
  /// `res` if OMPL succeeded; false otherwise. Logs the outcome.
  bool runOmplFallback(
      const planning_scene::PlanningSceneConstPtr& scene,
      const planning_interface::MotionPlanRequest& req,
      planning_interface::MotionPlanResponse& res);

  static constexpr size_t FRIDA_DOF = 6;
};

}

#endif