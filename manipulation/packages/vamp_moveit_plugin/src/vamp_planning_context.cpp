#include "vamp_moveit_plugin/vamp_planning_context.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_state/conversions.h>
#include "vamp_moveit_plugin/srv/vamp_plan.hpp"


#include <moveit/collision_detection/world.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>

namespace vamp_moveit_plugin
{

VampPlanningContext::VampPlanningContext(const std::string& name,
                                         const std::string& group_name,
                                         const moveit::core::RobotModelConstPtr& model,
                                         const rclcpp::Node::SharedPtr& node)
  : planning_interface::PlanningContext(name, group_name),
    node_(node),
    robot_model_(model)
{
  RCLCPP_INFO(node_->get_logger(), "VampPlanningContext initialized for: %s", getGroupName().c_str());
}

VampPlanningContext::~VampPlanningContext() {}
void VampPlanningContext::clear() {}


bool VampPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  const planning_interface::MotionPlanRequest& req = getMotionPlanRequest();
  
  
  std::vector<double> start_state;
  moveit::core::RobotState start_robot_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(req.start_state, start_robot_state);
  const moveit::core::JointModelGroup* jmg = start_robot_state.getJointModelGroup(getGroupName());
  start_robot_state.copyJointGroupPositions(jmg, start_state);

  
  std::vector<double> goal_state;
  if (!req.goal_constraints.empty() && !req.goal_constraints[0].joint_constraints.empty()) {
    for (const auto& jc : req.goal_constraints[0].joint_constraints) {
      goal_state.push_back(jc.position);
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "VAMP: No valid Goal Constraints provided.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  
  auto request = std::make_shared<vamp_moveit_plugin::srv::VampPlan::Request>();
  request->start_state = start_state;
  request->goal_state = goal_state;

  
  planning_scene::PlanningSceneConstPtr scene = getPlanningScene();
  collision_detection::WorldConstPtr world = scene->getWorld();

  for (const auto& object_id : world->getObjectIds()) {
    auto object = world->getObject(object_id);

    Eigen::Isometry3d global_pose = scene->getFrameTransform(object_id);

    for (size_t i = 0; i < object->shapes_.size(); ++i) {
      if (object->shapes_[i]->type == shapes::SPHERE) {
        const auto* s = static_cast<const shapes::Sphere*>(object->shapes_[i].get());

        request->sphere_centers_flat.push_back(global_pose.translation().x());
        request->sphere_centers_flat.push_back(global_pose.translation().y());
        request->sphere_centers_flat.push_back(global_pose.translation().z());
        request->sphere_radii.push_back(s->radius);
      } else if (object->shapes_[i]->type == shapes::BOX) {
        const auto* b = static_cast<const shapes::Box*>(object->shapes_[i].get());
        request->box_centers_flat.push_back(global_pose.translation().x());
        request->box_centers_flat.push_back(global_pose.translation().y());
        request->box_centers_flat.push_back(global_pose.translation().z());

        request->box_sizes_flat.push_back(b->size[0]);
        request->box_sizes_flat.push_back(b->size[1]);
        request->box_sizes_flat.push_back(b->size[2]);
      }
    }
  }

  
  auto temp_node = std::make_shared<rclcpp::Node>("vamp_client_temp_node");
  auto client = temp_node->create_client<vamp_moveit_plugin::srv::VampPlan>("plan_vamp_path");

  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "VAMP server not available.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  auto result_future = client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(temp_node, result_future, std::chrono::seconds(10)) != 
      rclcpp::FutureReturnCode::SUCCESS) 
  {
    RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for VAMP response.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
    return false;
  }

  auto result = result_future.get();
  if (!result->success) {
    RCLCPP_WARN(node_->get_logger(), "VAMP could not find a valid trajectory.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  
  size_t dof = start_state.size();
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  trajectory_msg.joint_trajectory.joint_names = jmg->getActiveJointModelNames();

  for (size_t i = 0; i < result->waypoints_flat.size(); i += dof) {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    for (size_t j = 0; j < dof; ++j) {
      pt.positions.push_back(result->waypoints_flat[i + j]);
    }
    pt.time_from_start = rclcpp::Duration::from_seconds((i / dof) * 0.1);
    trajectory_msg.joint_trajectory.points.push_back(pt);
  }

  res.trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());
  res.trajectory_->setRobotTrajectoryMsg(start_robot_state, trajectory_msg);
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  RCLCPP_INFO(node_->get_logger(), "VAMP planning succeeded, avoiding %ld spheres!", request->sphere_radii.size());
  return true;
}


bool VampPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse simple_res;
  bool success = solve(simple_res);

  if (success)
  {
    res.trajectory_.push_back(simple_res.trajectory_);
    res.description_.push_back("vamp_plan");
    res.processing_time_.push_back(simple_res.planning_time_);
    res.error_code_ = simple_res.error_code_;
  }
  else
  {
    res.error_code_ = simple_res.error_code_;
  }

  return success;
}

bool VampPlanningContext::terminate() { return true; }
bool VampPlanningContext::convertPlanningSceneToVamp(void*) const { return true; }
bool VampPlanningContext::convertShapeToVamp(const shapes::Shape*, const Eigen::Isometry3d&, void*) const { return true; }
bool VampPlanningContext::callVampPlanner(const std::vector<double>&, const std::vector<double>&, void*, std::vector<std::vector<double>>&) const { return true; }

} 