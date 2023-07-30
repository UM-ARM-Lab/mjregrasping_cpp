#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include <arc_utilities/eigen_typedefs.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace EigenHelpers;

class RRTPlanner {
 public:
  RRTPlanner();

  moveit_msgs::MotionPlanResponse plan(moveit_msgs::PlanningScene const &scene_msg, std::string const &group_name,
                                       std::map<std::string, Eigen::Vector3d> const &goal_positions);

  ros::NodeHandle nh;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader;
  std::shared_ptr<robot_model::RobotModel> robot_model;
  std::shared_ptr<planning_pipeline::PlanningPipeline> planning_pipeline;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
};