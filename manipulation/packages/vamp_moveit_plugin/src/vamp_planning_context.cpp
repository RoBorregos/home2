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
    std::shared_ptr<vamp_moveit_plugin::srv::VampPlan::Request>& request,
    const Eigen::Vector3d& goal_ee_position) const
{
  collision_detection::WorldConstPtr world = scene->getWorld();


  
  Eigen::Isometry3d vamp_frame_transform = Eigen::Isometry3d::Identity();
  try {
    vamp_frame_transform = scene->getFrameTransform("link_base").inverse();
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "Could not get link_base transform: %s", e.what());
  }
  RCLCPP_DEBUG(node_->get_logger(), "vamp_frame_transform translation: (%.3f, %.3f, %.3f)", vamp_frame_transform.translation().x(), vamp_frame_transform.translation().y(), vamp_frame_transform.translation().z());
  for (const auto& object_id : world->getObjectIds()) {
    auto object = world->getObject(object_id);
    Eigen::Isometry3d global_pose = scene->getFrameTransform(object_id);

    for (size_t i = 0; i < object->shapes_.size(); ++i) {
      Eigen::Isometry3d shape_pose = vamp_frame_transform * global_pose * object->shape_poses_[i];

      RCLCPP_DEBUG(node_->get_logger(), "Object %s: global=(%.3f,%.3f,%.3f) transformed=(%.3f,%.3f,%.3f)", object_id.c_str(), global_pose.translation().x(), global_pose.translation().y(), global_pose.translation().z(), shape_pose.translation().x(), shape_pose.translation().y(), shape_pose.translation().z());
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
          size_t goal_filtered = 0;
          const double ws_limit = 1.5;
          const double goal_clearance = 0.12;

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

              if ((pos - goal_ee_position).norm() < goal_clearance) {
                ++goal_filtered;
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
            "Octree '%s': %zu voxels extracted, %zu filtered near goal (res=%.3f)",
            object_id.c_str(), voxel_count, goal_filtered, octree->getResolution());
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
  moveit::core::RobotState start_robot_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(req.start_state, start_robot_state);
  const moveit::core::JointModelGroup* jmg =
      start_robot_state.getJointModelGroup(getGroupName());
  start_robot_state.copyJointGroupPositions(jmg, start_state);

  
  std::vector<double> goal_state;
  if (!req.goal_constraints.empty() &&
      !req.goal_constraints[0].joint_constraints.empty()) {
    for (const auto& jc : req.goal_constraints[0].joint_constraints) {
      goal_state.push_back(jc.position);
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "No valid goal constraints.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }


  auto request = std::make_shared<vamp_moveit_plugin::srv::VampPlan::Request>();
  request->start_state = start_state;
  request->goal_state = goal_state;


  moveit::core::RobotState goal_robot_state(robot_model_);
  goal_robot_state.setJointGroupPositions(jmg, goal_state);
  goal_robot_state.update();
  const std::string& ee_link = jmg->getLinkModelNames().back();
  Eigen::Vector3d goal_ee_position = goal_robot_state.getGlobalLinkTransform(ee_link).translation();

  planning_scene::PlanningSceneConstPtr scene = getPlanningScene();
  Eigen::Isometry3d vamp_frame_transform = Eigen::Isometry3d::Identity();
  try {
    vamp_frame_transform = scene->getFrameTransform("link_base").inverse();
  } catch (...) {}
  goal_ee_position = vamp_frame_transform * goal_ee_position;

  extractCollisionScene(scene, request, goal_ee_position);

  RCLCPP_INFO(node_->get_logger(),
    "VAMP request: %zu spheres, %zu boxes",
    request->sphere_radii.size(),
    request->box_sizes_flat.size() / 3);

  
  if (!vamp_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
      "VAMP server unavailable! Is vamp_server.py running?");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  
  auto result_future = vamp_client_->async_send_request(request);

  double timeout_s = std::max(10.0, req.allowed_planning_time);
  auto status = result_future.wait_for(
      std::chrono::milliseconds(static_cast<int>(timeout_s * 1000)));

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
      "VAMP timeout after %.1f s", timeout_s);
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
    return false;
  }

  auto result = result_future.get();

  if (!result->success) {
    RCLCPP_WARN(node_->get_logger(), "VAMP planning failed.");
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

  return true;
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