#include <bio_ik/bio_ik.h>
#include <mjregrasping/planning.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

constexpr auto LOGNAME = "planning";

namespace rvt = rviz_visual_tools;

constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }

RRTPlanner::RRTPlanner() {
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("hdt_michigan/robot_description");
  robot_model = robot_model_loader->getModel();

  nh.setParam("planning_plugin", "ompl_interface/OMPLPlanner");

  planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, nh);

  visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("robot_root");
  visual_tools->deleteAllMarkers();
}

moveit_msgs::MotionPlanResponse RRTPlanner::plan(moveit_msgs::PlanningScene const &scene_msg,
                                                 std::string const &group_name,
                                                 std::map<std::string, Eigen::Vector3d> const &goal_positions) {
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  planning_scene->setPlanningSceneMsg(scene_msg);

  auto state = planning_scene->getCurrentStateNonConst();

  state.update();  // update FK

  // Solve IK
  auto opts = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
  opts->replace = true;                       // needed to replace the default goals!!!
  opts->return_approximate_solution = false;  // optional
  for (auto const &[name, p] : goal_positions) {
    tf2::Vector3 position(p(0), p(1), p(2));
    opts->goals.emplace_back(std::make_unique<bio_ik::PositionGoal>(name, position));
  }

  const moveit::core::JointModelGroup *jmg = state.getJointModelGroup(group_name);

  // NOTE: could add collision checking to the IK callback here, but I don't think it's necessary because the RRT is
  //     already doing collision checking.
  moveit::core::GroupStateValidityCallbackFn state_valid_cb =
      [](moveit::core::RobotState *robot_state, const moveit::core::JointModelGroup *joint_group,
         const double *joint_group_variable_values) { return true; };

  planning_interface::MotionPlanRequest req;
  req.group_name = group_name;

  for (auto i{0}; i < 30; ++i) {
    state.setToRandomPositionsNearBy(jmg, state, 0.25);
    auto const ok = state.setFromIK(jmg,                            // joints to be used for IK
                                    EigenSTL::vector_Isometry3d(),  // this isn't used, goals are described in opts
                                    std::vector<std::string>(),     // names of the end-effector links
                                    0,                              // take values from YAML
                                    state_valid_cb, *opts);
    if (!ok) {
      continue;
    }

    visual_tools->publishRobotState(state, rvt::CYAN);

    moveit_msgs::Constraints joints_goal_constraint;
    for (auto const &n : jmg->getActiveJointModelNames()) {
      moveit_msgs::JointConstraint joint_constraint;
      joint_constraint.joint_name = n;
      joint_constraint.position = state.getVariablePosition(n);
      joint_constraint.tolerance_above = deg2rad(0.5);
      joint_constraint.tolerance_below = deg2rad(0.5);
      joint_constraint.weight = 1.0;
      joints_goal_constraint.joint_constraints.push_back(joint_constraint);
    }

    req.goal_constraints.push_back(joints_goal_constraint);
  }

  ROS_INFO_STREAM_NAMED(LOGNAME, "Planning with " << req.goal_constraints.size() << " goal constraints");

  planning_interface::MotionPlanResponse res;
  req.allowed_planning_time = 30.0;
  planning_pipeline->generatePlan(planning_scene, req, res);

  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);

  return res_msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_demo");

  std::string const group_name = "right_arm";

  auto planner = RRTPlanner();

  robot_state::RobotState state(planner.robot_model);
  // NOTE q could be passed in
  std::vector<double> q(state.getVariableCount(), 0.0);

  moveit_msgs::PlanningScene scene_msg;
  scene_msg.is_diff = false;
  scene_msg.robot_state.is_diff = false;
  moveit::core::robotStateToRobotStateMsg(state, scene_msg.robot_state);
  scene_msg.world.collision_objects.resize(1);
  auto &co = scene_msg.world.collision_objects[0];
  co.id = "box1";
  co.header.frame_id = "robot_root";
  co.pose.orientation.w = 1.0;
  co.primitive_poses.resize(1);
  auto &pose = co.primitive_poses[0];
  pose.orientation.w = 1.0;
  pose.position.x = 0.2;
  pose.position.y = 0.7;
  pose.position.z = 0.5;
  co.primitives.resize(1);
  auto &shape = co.primitives[0];
  shape.dimensions.resize(3);
  shape.dimensions[0] = 0.1;
  shape.dimensions[1] = 0.2;
  shape.dimensions[2] = 0.5;
  shape.type = shape_msgs::SolidPrimitive::BOX;

  auto scene_pub = planner.nh.advertise<moveit_msgs::PlanningScene>("scene_viz", 10);

  for (auto i = 0; i < 10; ++i) {
    scene_pub.publish(scene_msg);
    usleep(100000);
  }

  return 0;

  std::map<std::string, Eigen::Vector3d> goal_positions{
      //      {"left_tool", Eigen::Vector3d(-0.2, 0.8, 0.7)},
      {"right_tool", Eigen::Vector3d(0.2, 0.8, 0.7)},
  };

  auto const res_msg = planner.plan(scene_msg, group_name, goal_positions);

  moveit_msgs::DisplayTrajectory display_traj_msg;
  display_traj_msg.model_id = planner.robot_model->getName();
  display_traj_msg.trajectory_start = res_msg.trajectory_start;
  display_traj_msg.trajectory.push_back(res_msg.trajectory);

  for (auto i = 0; i < 10; ++i) {
    planner.visual_tools->publishTrajectoryPath(display_traj_msg);
    usleep(100000);
  }

  return 0;
}