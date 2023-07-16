#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  std::string const group_name = "left_side";

  auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("hdt_michigan/robot_description");

  std::string const robot_namespace = "hdt_michigan";

  auto robot_model = robot_model_loader->getModel();
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("whole_body");
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "home");

  auto planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, nh);

  auto display_publisher =
      nh.advertise<moveit_msgs::DisplayTrajectory>("hdt_michigan/move_group/display_planned_path", 10);
  moveit_msgs::DisplayTrajectory display_trajectory;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("robot_root");
  visual_tools.deleteAllMarkers();

  // Pose Goal
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "robot_root";
  pose.pose.position.x = -0.2;
  pose.pose.position.y = 0.6;
  pose.pose.position.z = 0.4;

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(-M_PI_2, 0, 0);
  myQuaternion = myQuaternion.normalize();
  pose.pose.orientation.x = myQuaternion.x();
  pose.pose.orientation.y = myQuaternion.y();
  pose.pose.orientation.z = myQuaternion.z();
  pose.pose.orientation.w = myQuaternion.w();

  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  req.group_name = group_name;
  auto pose_goal = kinematic_constraints::constructGoalConstraints("left_tool", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  planning_pipeline->generatePlan(planning_scene, req, res);

  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  std::cin.get();

  return 0;
}