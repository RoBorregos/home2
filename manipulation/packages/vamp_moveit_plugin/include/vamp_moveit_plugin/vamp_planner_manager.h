/*********************************************************************
 * Software License Agreement (MIT License)
 *
 *  Copyright (c) 2024 RoBorregos
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *********************************************************************/

/**
 * @file vamp_planner_manager.h
 * @brief MoveIt 2 planner manager implementation for VAMP
 * @author RoBorregos
 * @date 2024
 */

#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <map>

namespace vamp_moveit_plugin
{

/**
 * @class VampPlannerManager
 * @brief Implements the MoveIt 2 PlannerManager interface for VAMP planner
 * 
 * This class is responsible for:
 * - Initializing the VAMP planner plugin
 * - Creating planning contexts for motion planning requests
 * - Managing planner configuration and settings
 * - Reporting available planning algorithms
 */
class VampPlannerManager : public planning_interface::PlannerManager
{
public:
  /**
   * @brief Constructor
   */
  VampPlannerManager() = default;

  /**
   * @brief Destructor
   */
  ~VampPlannerManager() override = default;

  /**
   * @brief Initialize the planner manager
   * @param robot_model The robot kinematic model
   * @param node ROS 2 node for parameter access
   * @param parameter_namespace Namespace for planner parameters
   * @return true if initialization successful
   */
  bool initialize(const moveit::core::RobotModelConstPtr& robot_model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  /**
   * @brief Get description of the planner
   * @return String description
   */
  std::string getDescription() const override;

  /**
   * @brief Get list of available planning algorithms
   * @return Vector of algorithm names
   */
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  /**
   * @brief Create a planning context for a motion planning request
   * @param planning_scene Current planning scene with collision environment
   * @param req Motion planning request
   * @param error_code Output error code
   * @return Shared pointer to planning context
   */
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  /**
   * @brief Check if the planner can service the given request
   * @param req Motion planning request
   * @return true if request can be serviced
   */
  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

  /**
   * @brief Set planner configurations (optional)
   */
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override;

private:
  /// Robot kinematic model
  moveit::core::RobotModelConstPtr robot_model_;

  /// ROS 2 node for parameter access and logging
  rclcpp::Node::SharedPtr node_;

  /// Parameter namespace for this planner
  std::string parameter_namespace_;

  /// Planner configurations
  planning_interface::PlannerConfigurationMap planner_configs_;

  /// Default planner ID
  std::string default_planner_id_{ "VAMP_RRTConnect" };

  /// Planning group name (e.g., "arm")
  std::string planning_group_;

  /// Number of DOF for FRIDA arm (8: 6 revolute + 2 prismatic)
  static constexpr size_t FRIDA_DOF = 8;
};

}  // namespace vamp_moveit_plugin
