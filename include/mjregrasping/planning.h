#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
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

  /***
   *
   * @param scene_msg make a copy because we modify the ACM to match the SRDF
   * @param group_name
   * @param goal_positions
   * @return
   */
  moveit_msgs::MotionPlanResponse plan(moveit_msgs::PlanningScene scene_msg, std::string const &group_name,
                                       std::map<std::string, Eigen::Vector3d> const &goal_positions, bool viz = true,
                                       double const allowed_planning_time = 5.0, double const pos_noise = 0.001);

  planning_scene::PlanningScenePtr get_planning_scene(moveit_msgs::PlanningScene scene_msg,
                                                      robot_model::RobotModelPtr robot_model);

  bool is_state_valid(moveit_msgs::PlanningScene scene_msg);

  void display_result(moveit_msgs::MotionPlanResponse const &res_msg);

  ros::NodeHandle nh;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model::RobotModelPtr robot_model;
  planning_pipeline::PlanningPipelinePtr planning_pipeline;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  ros::Publisher ik_traj_pub;
  ros::Publisher sln_traj_pub;
  ros::Publisher scene_pub;
  trajectory_processing::IterativeParabolicTimeParameterization time_param_;
};

void seedOmpl(int seed);