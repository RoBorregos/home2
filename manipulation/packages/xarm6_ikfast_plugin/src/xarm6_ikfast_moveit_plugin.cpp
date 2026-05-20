/*
 * IKFast MoveIt2 plugin for xArm6
 *
 * Wraps the auto-generated IKFast analytical solver (xarm6_ikfast61.cpp)
 * into a MoveIt2 kinematics plugin.
 *
 * Based on the MoveIt2 IKFast plugin template.
 * Original solver from: google-research/pyreach (Apache 2.0)
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>

// IKFast includes
#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN
#include "xarm6_ikfast61.cpp"

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <algorithm>
#include <limits>
#include <random>
#include <vector>

namespace xarm6_ikfast_plugin
{

// Number of joints
static const int NUM_JOINTS = 6;

class IKFastKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  IKFastKinematicsPlugin() = default;

  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const moveit::core::RobotModel& robot_model,
                  const std::string& group_name,
                  const std::string& base_frame,
                  const std::vector<std::string>& tip_frames,
                  double search_discretization) override
  {
    node_ = node;
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

    const moveit::core::JointModelGroup* jmg = robot_model.getJointModelGroup(group_name);
    if (!jmg)
    {
      RCLCPP_ERROR(node_->get_logger(), "IKFast: joint model group '%s' not found", group_name.c_str());
      return false;
    }

    joint_names_ = jmg->getActiveJointModelNames();
    link_names_ = jmg->getLinkModelNames();

    if (joint_names_.size() != NUM_JOINTS)
    {
      RCLCPP_ERROR(node_->get_logger(), "IKFast: expected %d joints, got %zu", NUM_JOINTS, joint_names_.size());
      return false;
    }

    // Get joint limits
    for (const auto& jname : joint_names_)
    {
      const moveit::core::JointModel* jm = robot_model.getJointModel(jname);
      const auto& bounds = jm->getVariableBounds();
      if (!bounds.empty())
      {
        joint_min_.push_back(bounds[0].min_position_);
        joint_max_.push_back(bounds[0].max_position_);
      }
      else
      {
        joint_min_.push_back(-M_PI);
        joint_max_.push_back(M_PI);
      }
    }

    RCLCPP_INFO(node_->get_logger(), "IKFast plugin initialized for xArm6 (%zu joints)", joint_names_.size());
    return true;
  }

  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                     const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution,
                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    std::vector<double> vfree;
    std::vector<geometry_msgs::msg::Pose> poses = {ik_pose};
    std::vector<std::vector<double>> solutions;

    if (!searchPositionIK(poses[0], ik_seed_state, 0.05, solution, error_code, options))
    {
      return false;
    }
    return true;
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    const IKCallbackFn solution_callback = nullptr;
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code, options);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    const IKCallbackFn solution_callback = nullptr;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code, options);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code, options);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    // Convert pose to IKFast format (3x3 rotation + 3 translation)
    KDL::Frame frame;
    tf2::fromMsg(ik_pose, frame);

    IkReal eerot[9], eetrans[3];
    eetrans[0] = frame.p[0];
    eetrans[1] = frame.p[1];
    eetrans[2] = frame.p[2];

    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        eerot[i * 3 + j] = frame.M(i, j);

    // Call IKFast
    IkSolutionList<IkReal> ik_solutions;
    bool success = ComputeIk(eetrans, eerot, nullptr, ik_solutions);

    if (!success || ik_solutions.GetNumSolutions() == 0)
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    // Find the solution closest to the seed state that respects joint limits
    double best_dist = std::numeric_limits<double>::max();
    bool found_valid = false;

    for (size_t i = 0; i < ik_solutions.GetNumSolutions(); ++i)
    {
      const auto& sol = ik_solutions.GetSolution(i);
      std::vector<IkReal> values(NUM_JOINTS);
      std::vector<IkReal> free_values;
      sol.GetSolution(values.data(), free_values.data());

      // Normalize angles to [-pi, pi] and check limits
      std::vector<double> candidate(NUM_JOINTS);
      bool within_limits = true;

      for (int j = 0; j < NUM_JOINTS; ++j)
      {
        // Normalize to [-pi, pi]
        double angle = values[j];
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        // Also try angle +/- 2*pi if original is out of limits
        if (angle < joint_min_[j] || angle > joint_max_[j])
        {
          if (angle + 2.0 * M_PI <= joint_max_[j])
            angle += 2.0 * M_PI;
          else if (angle - 2.0 * M_PI >= joint_min_[j])
            angle -= 2.0 * M_PI;
          else
          {
            within_limits = false;
            break;
          }
        }
        candidate[j] = angle;
      }

      if (!within_limits)
        continue;

      // Check consistency limits
      if (!consistency_limits.empty())
      {
        bool consistent = true;
        for (int j = 0; j < NUM_JOINTS && j < (int)consistency_limits.size(); ++j)
        {
          if (std::abs(candidate[j] - ik_seed_state[j]) > consistency_limits[j])
          {
            consistent = false;
            break;
          }
        }
        if (!consistent)
          continue;
      }

      // Compute distance to seed
      double dist = 0;
      for (int j = 0; j < NUM_JOINTS; ++j)
      {
        double d = candidate[j] - ik_seed_state[j];
        dist += d * d;
      }

      if (dist < best_dist)
      {
        best_dist = dist;
        solution = candidate;
        found_valid = true;
      }
    }

    if (!found_valid)
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    // If callback provided, validate
    if (solution_callback)
    {
      solution_callback(ik_pose, solution, error_code);
      if (error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        return false;
    }

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }

  bool getPositionFK(const std::vector<std::string>& link_names,
                     const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override
  {
    if (joint_angles.size() != NUM_JOINTS)
      return false;

    IkReal eerot[9], eetrans[3];
    IkReal joints[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; ++i)
      joints[i] = joint_angles[i];

    ComputeFk(joints, eetrans, eerot);

    poses.resize(1);
    KDL::Frame frame;
    frame.p[0] = eetrans[0];
    frame.p[1] = eetrans[1];
    frame.p[2] = eetrans[2];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        frame.M(i, j) = eerot[i * 3 + j];

    poses[0] = tf2::toMsg(frame);
    return true;
  }

  const std::vector<std::string>& getJointNames() const override { return joint_names_; }
  const std::vector<std::string>& getLinkNames() const override { return link_names_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::vector<double> joint_min_;
  std::vector<double> joint_max_;
};

}  // namespace xarm6_ikfast_plugin

PLUGINLIB_EXPORT_CLASS(xarm6_ikfast_plugin::IKFastKinematicsPlugin, kinematics::KinematicsBase)
