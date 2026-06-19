#include "vamp_moveit_plugin/vamp_planning_context.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_state/conversions.h>
#include "vamp_moveit_plugin/srv/vamp_plan.hpp"

#include <moveit/collision_detection/world.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <octomap/octomap.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

namespace vamp_moveit_plugin
{





VampPlanningContext::VampPlanningContext(
    const std::string& name,
    const std::string& group_name,
    const moveit::core::RobotModelConstPtr& model,
    const rclcpp::Node::SharedPtr& node)
  : planning_interface::PlanningContext(name, group_name),
    node_(node),
    robot_model_(model)
{
  
  
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__node:=vamp_planning_client"});
  vamp_node_ = std::make_shared<rclcpp::Node>("vamp_planning_client", opts);

  
  vamp_client_ = vamp_node_->create_client<vamp_moveit_plugin::srv::VampPlan>("plan_vamp_path");

  
  vamp_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  vamp_executor_->add_node(vamp_node_);
  vamp_executor_thread_ = std::thread([this]() {
    vamp_executor_->spin();
  });

  RCLCPP_INFO(node_->get_logger(),
    "VampPlanningContext initialized for group: %s (dedicated executor thread started)",
    getGroupName().c_str());
}

VampPlanningContext::~VampPlanningContext()
{
  
  if (vamp_executor_) {
    vamp_executor_->cancel();
  }
  if (vamp_executor_thread_.joinable()) {
    vamp_executor_thread_.join();
  }
}

void VampPlanningContext::clear() {}





void VampPlanningContext::extractCollisionScene(
    const planning_scene::PlanningSceneConstPtr& scene,
    std::shared_ptr<vamp_moveit_plugin::srv::VampPlan::Request>& request) const
{
  collision_detection::WorldConstPtr world = scene->getWorld();


  
  // frida_real FK is relative to base_link (URDF root), NOT link_base.
  // Obstacles from MoveIt's planning scene are already in base_link frame.
  // No frame transform needed.
  for (const auto& object_id : world->getObjectIds()) {
    auto object = world->getObject(object_id);
    Eigen::Isometry3d global_pose = scene->getFrameTransform(object_id);

    for (size_t i = 0; i < object->shapes_.size(); ++i) {
      Eigen::Isometry3d shape_pose = global_pose * object->shape_poses_[i];
      switch (object->shapes_[i]->type) {

        case shapes::SPHERE: {
          const auto* s = static_cast<const shapes::Sphere*>(object->shapes_[i].get());
          Eigen::Vector3d pos = shape_pose.translation();
          request->sphere_centers_flat.push_back(pos.x());
          request->sphere_centers_flat.push_back(pos.y());
          request->sphere_centers_flat.push_back(pos.z());
          request->sphere_radii.push_back(s->radius);
          break;
        }

        case shapes::BOX: {
          const auto* b = static_cast<const shapes::Box*>(object->shapes_[i].get());
          
          Eigen::Vector3d half(b->size[0] / 2.0, b->size[1] / 2.0, b->size[2] / 2.0);
          Eigen::Matrix3d rot = shape_pose.rotation();
          Eigen::Vector3d aabb_half;
          for (int r = 0; r < 3; ++r) {
            aabb_half[r] = std::abs(rot(r, 0)) * half[0]
                         + std::abs(rot(r, 1)) * half[1]
                         + std::abs(rot(r, 2)) * half[2];
          }
          Eigen::Vector3d center = shape_pose.translation();
          request->box_centers_flat.push_back(center.x());
          request->box_centers_flat.push_back(center.y());
          request->box_centers_flat.push_back(center.z());
          request->box_sizes_flat.push_back(aabb_half.x() * 2.0);
          request->box_sizes_flat.push_back(aabb_half.y() * 2.0);
          request->box_sizes_flat.push_back(aabb_half.z() * 2.0);
          break;
        }

        case shapes::CYLINDER: {
          const auto* c = static_cast<const shapes::Cylinder*>(object->shapes_[i].get());
          double bounding_r = std::sqrt(c->radius * c->radius +
                                        (c->length / 2.0) * (c->length / 2.0));
          Eigen::Vector3d pos = shape_pose.translation();
          request->sphere_centers_flat.push_back(pos.x());
          request->sphere_centers_flat.push_back(pos.y());
          request->sphere_centers_flat.push_back(pos.z());
          request->sphere_radii.push_back(bounding_r);
          break;
        }

        case shapes::OCTREE: {
          const auto* oc = static_cast<const shapes::OcTree*>(object->shapes_[i].get());
          std::shared_ptr<const octomap::OcTree> octree = oc->octree;
          if (!octree) {
            RCLCPP_WARN(node_->get_logger(), "Null octree for '%s'", object_id.c_str());
            break;
          }

          size_t voxel_count = 0;
          const double ws_limit = 1.5;

          for (auto it = octree->begin_leafs(), end = octree->end_leafs();
               it != end; ++it) {
            if (octree->isNodeOccupied(*it)) {
              Eigen::Vector3d local_pos(it.getX(), it.getY(), it.getZ());
              Eigen::Vector3d pos = shape_pose * local_pos;

              if (std::abs(pos.x()) > ws_limit ||
                  std::abs(pos.y()) > ws_limit ||
                  pos.z() < -0.5 || pos.z() > 2.0) {
                continue;
              }

              double radius = it.getSize() / 2.0;
              request->sphere_centers_flat.push_back(pos.x());
              request->sphere_centers_flat.push_back(pos.y());
              request->sphere_centers_flat.push_back(pos.z());
              request->sphere_radii.push_back(radius);
              ++voxel_count;
            }
          }

          RCLCPP_INFO(node_->get_logger(),
            "Octree '%s': %zu voxels extracted (res=%.3f)",
            object_id.c_str(), voxel_count, octree->getResolution());
          break;
        }

        case shapes::MESH: {
          const auto* m = static_cast<const shapes::Mesh*>(object->shapes_[i].get());
          Eigen::Vector3d pos = shape_pose.translation();
          double max_d2 = 0.0;
          for (unsigned int v = 0; v < m->vertex_count; ++v) {
            double d = m->vertices[3*v]*m->vertices[3*v]
                     + m->vertices[3*v+1]*m->vertices[3*v+1]
                     + m->vertices[3*v+2]*m->vertices[3*v+2];
            if (d > max_d2) max_d2 = d;
          }
          request->sphere_centers_flat.push_back(pos.x());
          request->sphere_centers_flat.push_back(pos.y());
          request->sphere_centers_flat.push_back(pos.z());
          request->sphere_radii.push_back(std::sqrt(max_d2));
          RCLCPP_WARN_ONCE(node_->get_logger(),
            "Mesh '%s' approximated as bounding sphere", object_id.c_str());
          break;
        }

        default:
          RCLCPP_WARN_ONCE(node_->get_logger(),
            "Unsupported shape type %d for '%s'",
            object->shapes_[i]->type, object_id.c_str());
          break;
      }
    }
  }
}





bool VampPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  auto t_start = std::chrono::steady_clock::now();

  const planning_interface::MotionPlanRequest& req = getMotionPlanRequest();

  
  std::vector<double> start_state;
  // Seed from the current scene state so joints absent from req.start_state (partial or
  // is_diff requests, e.g. from RViz) keep real values instead of uninitialized garbage.
  moveit::core::RobotState start_robot_state = getPlanningScene()->getCurrentState();
  moveit::core::robotStateMsgToRobotState(req.start_state, start_robot_state);
  const moveit::core::JointModelGroup* jmg =
      start_robot_state.getJointModelGroup(getGroupName());
  start_robot_state.copyJointGroupPositions(jmg, start_state);

  
  std::vector<double> goal_state;
  if (!req.goal_constraints.empty() &&
      !req.goal_constraints[0].joint_constraints.empty()) {
    // Joint-space goal: use directly
    for (const auto& jc : req.goal_constraints[0].joint_constraints) {
      goal_state.push_back(jc.position);
    }
  } else if (!req.goal_constraints.empty() &&
             (!req.goal_constraints[0].position_constraints.empty() ||
              !req.goal_constraints[0].orientation_constraints.empty())) {
    // Cartesian goal: solve IK to get joint values
    RCLCPP_INFO(node_->get_logger(), "Cartesian goal detected, solving IK...");

    moveit::core::RobotState goal_robot_state(start_robot_state);
    const auto& gc = req.goal_constraints[0];

    std::string target_link = jmg->getLinkModelNames().back();
    if (!gc.position_constraints.empty()) {
      target_link = gc.position_constraints[0].link_name;
    } else if (!gc.orientation_constraints.empty()) {
      target_link = gc.orientation_constraints[0].link_name;
    }

    geometry_msgs::msg::Pose target_pose;
    if (!gc.position_constraints.empty()) {
      const auto& pc = gc.position_constraints[0];
      target_pose.position.x = pc.constraint_region.primitive_poses[0].position.x;
      target_pose.position.y = pc.constraint_region.primitive_poses[0].position.y;
      target_pose.position.z = pc.constraint_region.primitive_poses[0].position.z;
    }
    if (!gc.orientation_constraints.empty()) {
      target_pose.orientation = gc.orientation_constraints[0].orientation;
    } else {
      target_pose.orientation.w = 1.0;
    }

    // Try multiple IK solutions, pick one that's collision-free
    planning_scene::PlanningSceneConstPtr ik_scene = getPlanningScene();
    bool ik_valid = false;
    const int max_ik_attempts = 20;

    for (int attempt = 0; attempt < max_ik_attempts; ++attempt) {
      if (attempt > 0) {
        goal_robot_state.setToRandomPositions(jmg);
      }

      bool ik_found = goal_robot_state.setFromIK(jmg, target_pose, target_link, 0.05);
      if (!ik_found) continue;

      if (ik_scene->isStateValid(goal_robot_state, jmg->getName())) {
        goal_robot_state.copyJointGroupPositions(jmg, goal_state);
        RCLCPP_INFO(node_->get_logger(),
          "IK solved for '%s' (attempt %d/%d), %zu joints",
          target_link.c_str(), attempt + 1, max_ik_attempts, goal_state.size());
        ik_valid = true;
        break;
      }
    }

    if (!ik_valid) {
      RCLCPP_ERROR(node_->get_logger(),
        "No collision-free IK for '%s' at (%.3f, %.3f, %.3f) after %d attempts",
        target_link.c_str(),
        target_pose.position.x, target_pose.position.y, target_pose.position.z,
        max_ik_attempts);
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "No valid goal constraints.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }


  auto request = std::make_shared<vamp_moveit_plugin::srv::VampPlan::Request>();
  request->start_state = start_state;
  request->goal_state = goal_state;

  planning_scene::PlanningSceneConstPtr scene = getPlanningScene();
  extractCollisionScene(scene, request);

  RCLCPP_INFO(node_->get_logger(),
    "VAMP request: %zu spheres, %zu boxes",
    request->sphere_radii.size(),
    request->box_sizes_flat.size() / 3);

  
  if (!vamp_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
      "VAMP server unavailable! Is vamp_server.py running?");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    // Server gone is a config/process issue, not a planning issue — OMPL
    // can't help here, surface the failure to the caller.
    return false;
  }


  auto result_future = vamp_client_->async_send_request(request);

  double timeout_s = std::max(10.0, req.allowed_planning_time);
  auto status = result_future.wait_for(
      std::chrono::milliseconds(static_cast<int>(timeout_s * 1000)));

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
      "VAMP timeout after %.1f s — trying OMPL fallback", timeout_s);
    if (runOmplFallback(scene, req, res)) {
      return true;
    }
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
    return false;
  }

  auto result = result_future.get();

  if (!result->success) {
    // VAMP said no — could be start/goal in collision per its sphere model,
    // or no path found within iter budget. Either way the sphere model may
    // be wrong; let OMPL decide with the actual mesh geometry.
    RCLCPP_WARN(node_->get_logger(),
      "VAMP planning failed — trying OMPL fallback");
    if (runOmplFallback(scene, req, res)) {
      return true;
    }
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

  res.trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(
      robot_model_, getGroupName());
  res.trajectory_->setRobotTrajectoryMsg(start_robot_state, trajectory_msg);

  
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  double max_vel = req.max_velocity_scaling_factor > 0.0 ?
                   req.max_velocity_scaling_factor : 1.0;
  double max_acc = req.max_acceleration_scaling_factor > 0.0 ?
                   req.max_acceleration_scaling_factor : 1.0;

  if (!totg.computeTimeStamps(*res.trajectory_, max_vel, max_acc)) {
    RCLCPP_WARN(node_->get_logger(),
      "TOTG failed — trajectory will have uniform timing.");
  }

  
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - t_start).count() / 1000.0;

  res.planning_time_ = elapsed_ms / 1000.0;
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  RCLCPP_INFO(node_->get_logger(),
    "VAMP SUCCESS | %.1f ms | %zu waypoints",
    elapsed_ms, result->waypoints_flat.size() / dof);

  // Post-validate the VAMP trajectory against the actual mesh-based collision
  // model. VAMP's sphere approximation can miss collisions FCL catches, so
  // any trajectory we return must survive the same check MoveIt's pipeline
  // does. On failure, fall back to OMPL (if installed) so the caller always
  // gets a valid plan when one exists.
  if (!validateTrajectoryAgainstScene(*res.trajectory_, scene)) {
    RCLCPP_WARN(node_->get_logger(),
      "VAMP plan failed FCL post-validation against current PlanningScene");
    res.trajectory_.reset();
    if (runOmplFallback(scene, req, res)) {
      return true;
    }
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  return true;
}

bool VampPlanningContext::validateTrajectoryAgainstScene(
    const robot_trajectory::RobotTrajectory& traj,
    const planning_scene::PlanningSceneConstPtr& scene) const
{
  // isPathValid checks each waypoint against scene collisions including
  // attached objects, mimicking the post-validation MoveIt's pipeline runs.
  if (scene->isPathValid(traj, getGroupName(), /*verbose=*/false)) {
    return true;
  }
  return false;
}

bool VampPlanningContext::runOmplFallback(
    const planning_scene::PlanningSceneConstPtr& scene,
    const planning_interface::MotionPlanRequest& req,
    planning_interface::MotionPlanResponse& res)
{
  if (!ompl_manager_) {
    RCLCPP_WARN(node_->get_logger(),
      "No OMPL fallback installed; returning failure to caller");
    return false;
  }

  // RRTConnect is stochastic, and TOTG can densify a path enough that
  // interpolated states graze obstacles that OMPL's own motion validator
  // missed (most often link/sensor parts mounted at awkward angles like the
  // ZED). Retry until we get a plan that survives our post-validation, or
  // give up after a bounded number of attempts so we never hang.
  constexpr int kMaxAttempts = 4;
  auto t_start = std::chrono::steady_clock::now();

  // OMPL picks the configured default planner when planner_id is empty.
  // Copy the request so the caller's view stays intact.
  planning_interface::MotionPlanRequest ompl_req = req;
  ompl_req.planner_id.clear();

  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    RCLCPP_INFO(node_->get_logger(),
      "Falling back to OMPL (attempt %d/%d)...", attempt, kMaxAttempts);

    moveit_msgs::msg::MoveItErrorCodes ec;
    auto ompl_ctx = ompl_manager_->getPlanningContext(scene, ompl_req, ec);
    if (!ompl_ctx) {
      RCLCPP_ERROR(node_->get_logger(),
        "OMPL fallback: getPlanningContext failed (code %d)", ec.val);
      return false;
    }

    planning_interface::MotionPlanResponse attempt_res;
    if (!ompl_ctx->solve(attempt_res)) {
      RCLCPP_WARN(node_->get_logger(),
        "OMPL attempt %d: solve() failed (code %d)",
        attempt, attempt_res.error_code_.val);
      continue;
    }

    if (!attempt_res.trajectory_) {
      RCLCPP_WARN(node_->get_logger(),
        "OMPL attempt %d: solve succeeded but trajectory is null", attempt);
      continue;
    }

    // OMPL returns a geometric path (no timing). Apply TOTG so the
    // controller accepts it, then validate the densified result against
    // the actual mesh model — that's the check the outer pipeline will
    // run anyway, so failing it here means a rejection downstream too.
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    double max_vel = req.max_velocity_scaling_factor > 0.0 ?
                     req.max_velocity_scaling_factor : 1.0;
    double max_acc = req.max_acceleration_scaling_factor > 0.0 ?
                     req.max_acceleration_scaling_factor : 1.0;
    if (!totg.computeTimeStamps(*attempt_res.trajectory_, max_vel, max_acc)) {
      RCLCPP_WARN(node_->get_logger(),
        "OMPL attempt %d: TOTG failed — skipping", attempt);
      continue;
    }

    if (!validateTrajectoryAgainstScene(*attempt_res.trajectory_, scene)) {
      RCLCPP_WARN(node_->get_logger(),
        "OMPL attempt %d: TOTG-densified trajectory has a colliding state "
        "(likely a sensor link grazing on interpolated motion); retrying",
        attempt);
      continue;
    }

    // Valid plan — copy to caller's response and report.
    res.trajectory_ = attempt_res.trajectory_;
    res.planning_time_ = attempt_res.planning_time_;
    res.error_code_ = attempt_res.error_code_;

    auto ms = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t_start).count() / 1000.0;
    RCLCPP_INFO(node_->get_logger(),
      "OMPL fallback SUCCESS | attempt %d/%d | %.1f ms total | %zu waypoints",
      attempt, kMaxAttempts, ms, res.trajectory_->getWayPointCount());
    return true;
  }

  auto ms = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - t_start).count() / 1000.0;
  RCLCPP_ERROR(node_->get_logger(),
    "OMPL fallback FAILED after %d attempts | %.1f ms total — caller will "
    "receive PLANNING_FAILED. The planned path keeps grazing geometry FCL "
    "rejects; consider increasing OMPL state_validation_resolution or "
    "checking for misaligned sensor links in the URDF.",
    kMaxAttempts, ms);
  return false;
}

bool VampPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse simple_res;
  bool success = solve(simple_res);
  if (success) {
    res.trajectory_.push_back(simple_res.trajectory_);
    res.description_.push_back("vamp_plan");
    res.processing_time_.push_back(simple_res.planning_time_);
    res.error_code_ = simple_res.error_code_;
  } else {
    res.error_code_ = simple_res.error_code_;
  }
  return success;
}

bool VampPlanningContext::terminate()
{
  terminate_ = true;
  return true;
}


bool VampPlanningContext::convertPlanningSceneToVamp(void*) const { return true; }
bool VampPlanningContext::convertShapeToVamp(
    const shapes::Shape*, const Eigen::Isometry3d&, void*) const { return true; }
bool VampPlanningContext::callVampPlanner(
    const std::vector<double>&, const std::vector<double>&,
    void*, std::vector<std::vector<double>>&) const { return true; }

}