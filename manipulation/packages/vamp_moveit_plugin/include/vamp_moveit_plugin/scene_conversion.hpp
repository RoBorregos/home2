#pragma once

#include <string>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "vamp_moveit_plugin/srv/vamp_plan.hpp"

namespace vamp_moveit_plugin
{
/// Copies the world collision objects and the octomap of the scene into the request, in the planning frame.
void addWorldObstacles(const planning_scene::PlanningScene& scene, srv::VampPlan::Request& request);

/// Copies the bodies attached to the robot into the request, as spheres relative to the end effector link.
void addAttachedBodies(const moveit::core::RobotState& state, const std::string& end_effector_link,
                       srv::VampPlan::Request& request);

}  // namespace vamp_moveit_plugin
