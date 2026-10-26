#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <rclcpp/logger.hpp>

#include "vamp_moveit_plugin/vamp_client.hpp"

namespace vamp_moveit_plugin
{
/// Settings read from the pipeline namespace (e.g. "vamp.request_timeout").
struct VampPlannerParams
{
  std::string service_name{ "/vamp_server/plan" };
  std::string end_effector_link{ "link_eef" };
  double request_timeout{ 2.0 };
  int fallback_attempts{ 4 };
};

/// Plans with VAMP, checks the result against the full PlanningScene and falls back to OMPL when VAMP cannot deliver.
class VampPlanningContext : public planning_interface::PlanningContext
{
public:
  VampPlanningContext(const std::string& name, const std::string& group, const VampPlannerParams& params,
                      std::shared_ptr<const VampClient> client,
                      std::shared_ptr<planning_interface::PlannerManager> fallback, const rclcpp::Logger& logger);

  void solve(planning_interface::MotionPlanResponse& res) override;
  void solve(planning_interface::MotionPlanDetailedResponse& res) override;
  bool terminate() override;
  void clear() override;

private:
  /// Samples a collision-free state that satisfies the goal constraints of the request before the deadline.
  std::optional<moveit::core::RobotState> sampleGoal(const moveit::core::RobotState& start,
                                                     std::chrono::steady_clock::time_point deadline) const;

  /// Returns a VAMP path that passes finalize(), or nullptr after logging why there is none.
  robot_trajectory::RobotTrajectoryPtr planWithVamp(const moveit::core::RobotState& start,
                                                    const moveit::core::RobotState& goal) const;

  /// Asks the VAMP server for a path between two states; returns nullptr when it has none.
  robot_trajectory::RobotTrajectoryPtr requestPath(const moveit::core::RobotState& start,
                                                   const moveit::core::RobotState& goal) const;

  /// Plans the original request with OMPL until a result passes finalize().
  void planWithFallback(planning_interface::MotionPlanResponse& res) const;

  /// Time-parameterizes the path and checks it the same way MoveIt's ValidateSolution adapter will.
  bool finalize(robot_trajectory::RobotTrajectory& trajectory) const;

  VampPlannerParams params_;
  std::shared_ptr<const VampClient> client_;
  std::shared_ptr<planning_interface::PlannerManager> fallback_;
  rclcpp::Logger logger_;
};

}  // namespace vamp_moveit_plugin
