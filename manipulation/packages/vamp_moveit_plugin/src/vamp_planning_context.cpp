#include "vamp_moveit_plugin/vamp_planning_context.hpp"

#include <chrono>

#include <moveit/constraint_samplers/constraint_sampler_manager.hpp>
#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
#include <moveit/utils/message_checks.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "vamp_moveit_plugin/scene_conversion.hpp"

namespace vamp_moveit_plugin
{
using moveit_msgs::msg::MoveItErrorCodes;

VampPlanningContext::VampPlanningContext(const std::string& name, const std::string& group,
                                         const VampPlannerParams& params, std::shared_ptr<const VampClient> client,
                                         std::shared_ptr<planning_interface::PlannerManager> fallback,
                                         const rclcpp::Logger& logger)
  : planning_interface::PlanningContext(name, group)
  , params_(params)
  , client_(std::move(client))
  , fallback_(std::move(fallback))
  , logger_(logger)
{
}

void VampPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  const auto start_time = std::chrono::steady_clock::now();
  const auto& req = getMotionPlanRequest();
  const auto& scene = getPlanningScene();

  moveit::core::RobotState start = scene->getCurrentState();
  moveit::core::robotStateMsgToRobotState(scene->getTransforms(), req.start_state, start);
  start.update();
  moveit::core::robotStateToRobotStateMsg(start, res.start_state);

  if (!moveit::core::isEmpty(req.path_constraints))
  {
    RCLCPP_INFO(logger_, "VAMP cannot honor path constraints, planning with OMPL");
    planWithFallback(res);
  }
  else if (const auto goal = sampleGoal(start, start_time + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                              std::chrono::duration<double>(req.allowed_planning_time)));
           !goal)
  {
    // OMPL samples goals with the same sampler, so falling back would fail the same way
    RCLCPP_WARN(logger_, "No valid goal state within %.2f s", req.allowed_planning_time);
    res.planner_id = "vamp";
    res.error_code = MoveItErrorCodes::NO_IK_SOLUTION;
  }
  else if (auto trajectory = planWithVamp(start, *goal))
  {
    res.trajectory = trajectory;
    res.planner_id = "vamp";
    res.error_code = MoveItErrorCodes::SUCCESS;
  }
  else
  {
    RCLCPP_WARN(logger_, "Falling back to OMPL");
    planWithFallback(res);
  }

  res.planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
  RCLCPP_INFO(logger_, "%s plan %s in %.1f ms", res.planner_id.c_str(), res.error_code ? "found" : "failed",
              res.planning_time * 1000.0);
}

void VampPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse simple_res;
  solve(simple_res);
  res.error_code.val = simple_res.error_code.val;
  res.planner_id = simple_res.planner_id;
  if (simple_res.trajectory)
  {
    res.trajectory.push_back(simple_res.trajectory);
    res.description.push_back("plan");
    res.processing_time.push_back(simple_res.planning_time);
  }
}

bool VampPlanningContext::terminate()
{
  return true;
}

void VampPlanningContext::clear()
{
}

std::optional<moveit::core::RobotState>
VampPlanningContext::sampleGoal(const moveit::core::RobotState& start,
                                std::chrono::steady_clock::time_point deadline) const
{
  const auto& scene = getPlanningScene();
  for (const auto& constraints : getMotionPlanRequest().goal_constraints)
  {
    const auto sampler =
        constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(scene, group_, constraints);
    if (!sampler)
    {
      continue;
    }
    sampler->setGroupStateValidityCallback(
        [&scene](moveit::core::RobotState* state, const moveit::core::JointModelGroup* group, const double* values) {
          state->setJointGroupPositions(group, values);
          state->update();
          return scene->isStateValid(*state, group->getName());
        });

    kinematic_constraints::KinematicConstraintSet goal_constraints(scene->getRobotModel());
    goal_constraints.add(constraints, scene->getTransforms());

    moveit::core::RobotState goal(start);
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (sampler->sample(goal, start, 1))
      {
        goal.update();
        if (scene->isStateValid(goal, group_) && goal_constraints.decide(goal).satisfied)
        {
          return goal;
        }
      }
    }
  }
  return std::nullopt;
}

robot_trajectory::RobotTrajectoryPtr VampPlanningContext::planWithVamp(const moveit::core::RobotState& start,
                                                                       const moveit::core::RobotState& goal) const
{
  auto trajectory = requestPath(start, goal);
  if (trajectory && !finalize(*trajectory))
  {
    RCLCPP_WARN(logger_, "VAMP path collides in the full PlanningScene");
    return nullptr;
  }
  return trajectory;
}

robot_trajectory::RobotTrajectoryPtr VampPlanningContext::requestPath(const moveit::core::RobotState& start,
                                                                      const moveit::core::RobotState& goal) const
{
  if (!client_->isReady())
  {
    RCLCPP_ERROR(logger_, "VAMP server is not running on '%s'", params_.service_name.c_str());
    return nullptr;
  }

  const auto& scene = getPlanningScene();
  const auto* group = scene->getRobotModel()->getJointModelGroup(group_);

  VampClient::Request request;
  request.joint_names = group->getActiveJointModelNames();
  start.copyJointGroupPositions(group, request.start);
  goal.copyJointGroupPositions(group, request.goal);
  request.start_end_effector_pose = tf2::toMsg(start.getGlobalLinkTransform(params_.end_effector_link));
  addWorldObstacles(*scene, request);
  addAttachedBodies(start, params_.end_effector_link, request);

  const auto response = client_->plan(request, std::chrono::duration<double>(params_.request_timeout));
  if (!response)
  {
    RCLCPP_WARN(logger_, "VAMP server did not answer within %.1f s", params_.request_timeout);
    return nullptr;
  }
  if (!response->success)
  {
    RCLCPP_WARN(logger_, "VAMP found no path: %s", response->message.c_str());
    return nullptr;
  }

  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(scene->getRobotModel(), group);
  moveit::core::RobotState waypoint(start);
  for (const auto& point : response->trajectory.points)
  {
    waypoint.setVariablePositions(response->trajectory.joint_names, point.positions);
    waypoint.update();
    trajectory->addSuffixWayPoint(waypoint, 0.0);
  }
  return trajectory;
}

void VampPlanningContext::planWithFallback(planning_interface::MotionPlanResponse& res) const
{
  res.planner_id = "ompl_fallback";
  if (!fallback_)
  {
    RCLCPP_ERROR(logger_, "OMPL fallback is not available");
    res.error_code = MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  // RRTConnect is random, so a path whose timed version grazes an obstacle is replanned rather than rejected
  for (int attempt = 1; attempt <= params_.fallback_attempts; ++attempt)
  {
    MoveItErrorCodes error_code;
    const auto context = fallback_->getPlanningContext(getPlanningScene(), getMotionPlanRequest(), error_code);
    if (!context)
    {
      res.error_code = error_code;
      return;
    }
    context->solve(res);
    res.planner_id = "ompl_fallback";
    if (!res.error_code)
    {
      return;
    }
    if (finalize(*res.trajectory))
    {
      return;
    }
    RCLCPP_WARN(logger_, "OMPL path collides once time-parameterized (attempt %d/%d)", attempt,
                params_.fallback_attempts);
  }
  res.trajectory.reset();
  res.error_code = MoveItErrorCodes::INVALID_MOTION_PLAN;
}

bool VampPlanningContext::finalize(robot_trajectory::RobotTrajectory& trajectory) const
{
  const auto& req = getMotionPlanRequest();
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(trajectory, req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor))
  {
    return false;
  }
  return getPlanningScene()->isPathValid(trajectory, req.path_constraints, group_);
}

}  // namespace vamp_moveit_plugin
