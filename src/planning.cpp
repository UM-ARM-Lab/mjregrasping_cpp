#include <bio_ik/bio_ik.h>
#include <mjregrasping/planning.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

//    std::vector<double> out;
//    }
//    ROS_DEBUG_STREAM_NAMED("BIO_IK", "ok? " << ok);
//    ROS_DEBUG_STREAM_NAMED("BIO_IK", "q " << out);
//
//    if (!ok) {
//      return {};
//    }
//
//    return {out};
//  }

int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_demo");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  std::string const group_name = "whole_body";

  auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("hdt_michigan/robot_description");

  auto robot_model = robot_model_loader->getModel();

  nh.setParam("planning_plugin", "ompl_interface/OMPLPlanner");

  auto planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, nh);

  auto display_publisher =
      nh.advertise<moveit_msgs::DisplayTrajectory>("hdt_michigan/move_group/display_planned_path", 10);
  moveit_msgs::DisplayTrajectory display_trajectory;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("robot_root");
  visual_tools.deleteAllMarkers();

  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);

  // Solve IK
  auto opts = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
  opts->replace = true;                       // needed to replace the default goals!!!
  opts->return_approximate_solution = false;  // optional
  opts->goals.emplace_back(std::make_unique<bio_ik::PositionGoal>("left_tool", tf2::Vector3(0.8, 0.2, 0.8)));
  opts->goals.emplace_back(std::make_unique<bio_ik::PositionGoal>("right_tool", tf2::Vector3(0.8, -0.2, 0.7)));

  robot_state::RobotState state(robot_model);
  // NOTE q could be passed in
  std::vector<double> q(state.getVariableCount(), 0.0);
  planning_scene->getCurrentStateNonConst().setVariablePositions(q);
  state.update();  // update FK

  const moveit::core::JointModelGroup *jmg = state.getJointModelGroup(group_name);

  moveit::core::GroupStateValidityCallbackFn state_valid_cb =
      [](moveit::core::RobotState *robot_state, const moveit::core::JointModelGroup *joint_group,
         const double *joint_group_variable_values) { return true; };

  planning_interface::MotionPlanRequest req;
  req.group_name = group_name;

  for (auto i{0}; i < 1; ++i) {
    auto const ok = state.setFromIK(jmg,                            // joints to be used for IK
                                    EigenSTL::vector_Isometry3d(),  // this isn't used, goals are described in opts
                                    std::vector<std::string>(),     // names of the end-effector links
                                    0,                              // take values from YAML
                                    state_valid_cb, *opts);
    if (!ok) {
      continue;
    }

    visual_tools.publishRobotState(state, rvt::CYAN);

    moveit_msgs::Constraints joints_goal_constraint;
    for (auto const &n : jmg->getActiveJointModelNames()) {
      moveit_msgs::JointConstraint joint_constraint;
      joint_constraint.joint_name = n;
      joint_constraint.position = state.getVariablePosition(n);
      joint_constraint.tolerance_above = 0.01;
      joint_constraint.tolerance_below = 0.01;
      joint_constraint.weight = 1.0;
      joints_goal_constraint.joint_constraints.push_back(joint_constraint);
    }

    req.goal_constraints.push_back(joints_goal_constraint);
  }

  ROS_INFO_STREAM("Planning with " << req.goal_constraints.size() << " goals");

  planning_interface::MotionPlanResponse res;
  req.allowed_planning_time = 30.0;
  planning_pipeline->generatePlan(planning_scene, req, res);

  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);

  moveit_msgs::DisplayTrajectory display_traj_msg;
  display_traj_msg.model_id = robot_model->getName();
  display_traj_msg.trajectory_start = res_msg.trajectory_start;
  display_traj_msg.trajectory.push_back(res_msg.trajectory);

  for (auto i = 0; i < 10; ++i) {
    display_publisher.publish(display_traj_msg);
    usleep(100000);
  }

  return 0;
}