#include <bio_ik/bio_ik.h>
#include <mjregrasping/planning.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <arc_utilities/moveit_ostream_operators.hpp>
#include <arc_utilities/moveit_pose_type.hpp>
#include <arc_utilities/ostream_operators.hpp>
#include <ompl/util/RandomNumbers.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

constexpr auto LOGNAME = "planning";

namespace rvt = rviz_visual_tools;

constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }

void seedOmpl(int seed) { ompl::RNG::setSeed(seed); }

void wait_for_connection(ros::Publisher &pub) {
  auto const t0 = ros::Time::now();
  while (pub.getNumSubscribers() < 1) {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Waiting for subscriber on topic " << pub.getTopic());
    auto const dt = ros::Time::now() - t0;
    if (dt > ros::Duration(5)) {
      ROS_INFO_STREAM_NAMED(LOGNAME, "Gave up!");
      break;
    }
    ros::Duration(0.1).sleep();
  }
}

RRTPlanner::RRTPlanner() {
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("hdt_michigan/robot_description");
  robot_model = robot_model_loader->getModel();

  // This normally gets set by ompl_planning.yaml, but we set it here in case it's not set
  nh.setParam("planning_plugin", "ompl_interface/OMPLPlanner");

  planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, nh);

  visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("robot_root");
  visual_tools->deleteAllMarkers();

  ik_traj_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("ik_traj", 10);
  sln_traj_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 10);
  scene_pub = nh.advertise<moveit_msgs::PlanningScene>("scene_viz", 10);
}

bool RRTPlanner::is_state_valid(moveit_msgs::PlanningScene scene_msg) {
  auto planning_scene = get_planning_scene(scene_msg, robot_model);
  auto const &state = planning_scene->getCurrentState();
  auto const is_valid = planning_scene->isStateValid(state);
  if (!is_valid){
    collision_detection::CollisionRequest collisionRequest;
    collisionRequest.contacts = true;
    collisionRequest.max_contacts = 1;
    collisionRequest.max_contacts_per_pair = 1;
    collision_detection::CollisionResult collisionResult;
    planning_scene->checkCollision(collisionRequest, collisionResult, state);
    ROS_DEBUG_STREAM_NAMED(LOGNAME + ".check_collision", "Collision Result: " << collisionResult);
    return false;
  }
  return true;
}

planning_scene::PlanningScenePtr RRTPlanner::get_planning_scene(moveit_msgs::PlanningScene scene_msg,
                                                                robot_model::RobotModelPtr robot_model) {
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  // The planning scene will automatically pull the ACM from the SRDF which is what we want,
  // and to avoid overwriting it with a blank ACM we first copy the existing ACM into the scene.
  auto const acm = planning_scene->getAllowedCollisionMatrix();
  acm.getMessage(scene_msg.allowed_collision_matrix);
  planning_scene->setPlanningSceneMsg(scene_msg);
  return planning_scene;
}
moveit_msgs::MotionPlanResponse RRTPlanner::plan(moveit_msgs::PlanningScene scene_msg, std::string const &group_name,
                                                 std::map<std::string, Eigen::Vector3d> const &goal_positions, bool viz,
                                                 double allowed_planning_time, double pos_noise, int max_ik_attempts,
                                                 int max_ik_solutions, double joint_noise) {
  auto const planning_scene = get_planning_scene(scene_msg, robot_model);
  auto ik_state = planning_scene->getCurrentStateNonConst();
  ik_state.update();  // update FK

  moveit_msgs::PlanningScene scene_msg_viz;
  planning_scene->getPlanningSceneMsg(scene_msg_viz);
  if (viz) {
    //    wait_for_connection(scene_pub);
    scene_pub.publish(scene_msg_viz);
  }

  const moveit::core::JointModelGroup *jmg = ik_state.getJointModelGroup(group_name);

  // check that the tool names are in the group
  auto const names = jmg->getLinkModelNames();
  for (auto const &[name, p] : goal_positions) {
    if (name == jmg->getEndEffectorName()) {
      continue;
    }
    if (std::find(names.cbegin(), names.cend(), name) == names.cend()) {
      ROS_FATAL_STREAM_NAMED(LOGNAME, "Tool name " << name << " not in group " << group_name);
      throw std::runtime_error("Tool name not in group");
    }
  }

  planning_interface::MotionPlanRequest req;
  req.group_name = group_name;

  auto const &ik_t0 = ros::Time::now();
  moveit_msgs::RobotTrajectory ik_as_traj_msg;
  ik_as_traj_msg.joint_trajectory.joint_names = jmg->getActiveJointModelNames();

  moveit::core::GroupStateValidityCallbackFn state_valid_cb = [&](moveit::core::RobotState *robot_state,
                                                                  const moveit::core::JointModelGroup *joint_group,
                                                                  const double *joint_group_variable_values) {
    robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
    robot_state->update();
    return planning_scene->isStateValid(*robot_state);
  };

  for (auto i{0}; i < max_ik_attempts; ++i) {
    // Solve IK
    auto opts = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
    opts->replace = true;                       // needed to replace the default goals!!!
    opts->return_approximate_solution = false;  // optional
    opts->goals.emplace_back(std::make_unique<bio_ik::MinimalDisplacementGoal>(0.1));
    for (auto const &[name, p] : goal_positions) {
      Eigen::Vector3d const noisy_p = p.array() + (Eigen::Vector3d::Random().array() * pos_noise);
      tf2::Vector3 position(noisy_p(0), noisy_p(1), noisy_p(2));
      opts->goals.emplace_back(std::make_unique<bio_ik::PositionGoal>(name, position));
    }

    ik_state.setToRandomPositionsNearBy(jmg, ik_state, joint_noise);
    auto const ok = ik_state.setFromIK(jmg,                            // joints to be used for IK
                                    EigenSTL::vector_Isometry3d(),  // this isn't used, goals are described in opts
                                    std::vector<std::string>(),     // names of the end-effector links
                                    0,                              // take values from YAML
                                    state_valid_cb, *opts);

    if (!ok) {
      continue;
    }

    if (req.goal_constraints.size() >= max_ik_solutions) {
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Exiting early");
      break;
    }

    moveit_msgs::Constraints joints_goal_constraint;
    trajectory_msgs::JointTrajectoryPoint ik_as_traj_point_msg;
    for (auto const &n : jmg->getActiveJointModelNames()) {
      moveit_msgs::JointConstraint joint_constraint;
      joint_constraint.joint_name = n;
      joint_constraint.position = ik_state.getVariablePosition(n);
      joint_constraint.tolerance_above = deg2rad(1.5);
      joint_constraint.tolerance_below = deg2rad(1.5);
      joint_constraint.weight = 1.0;
      joints_goal_constraint.joint_constraints.push_back(joint_constraint);
      ik_as_traj_point_msg.positions.push_back(joint_constraint.position);
    }

    req.goal_constraints.push_back(joints_goal_constraint);

    ik_as_traj_point_msg.time_from_start = ros::Duration(0.1 * i);
    ik_as_traj_msg.joint_trajectory.points.push_back(ik_as_traj_point_msg);
  }

  // Add a path constraint so that no joint can change by more than PI from it's current position
  auto const &initial_state = planning_scene->getCurrentState();
  moveit_msgs::Constraints path_constraint;
  for (auto const &n : jmg->getActiveJointModelNames()) {
    moveit_msgs::JointConstraint joint_constraint;
    joint_constraint.joint_name = n;
    joint_constraint.position = initial_state.getVariablePosition(n);
    joint_constraint.tolerance_above = deg2rad(180);
    joint_constraint.tolerance_below = deg2rad(180);
    joint_constraint.weight = 1.0;
    path_constraint.joint_constraints.push_back(joint_constraint);
  }
  req.path_constraints = path_constraint;

  moveit_msgs::DisplayTrajectory disp_ik_as_traj_msg;
  disp_ik_as_traj_msg.model_id = robot_model->getName();
  disp_ik_as_traj_msg.trajectory.push_back(ik_as_traj_msg);
  disp_ik_as_traj_msg.trajectory_start = scene_msg.robot_state;

  if (viz) {
    //    wait_for_connection(ik_traj_pub);
    ik_traj_pub.publish(disp_ik_as_traj_msg);
  }

  auto const &ik_t1 = ros::Time::now();
  auto const &ik_dt = ik_t1 - ik_t0;
  ROS_INFO_STREAM_NAMED(LOGNAME + ".perf", "IK took " << ik_dt.toSec() << " seconds");

  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Found " << req.goal_constraints.size() << " IK solutions.");
  if (req.goal_constraints.empty()) {
    moveit_msgs::MotionPlanResponse res_msg;
    res_msg.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return res_msg;
  }
  ROS_INFO_STREAM_NAMED(LOGNAME, "Planning with " << req.goal_constraints.size() << " goal constraints");

  planning_interface::MotionPlanResponse res;
  req.allowed_planning_time = allowed_planning_time;
  planning_pipeline->generatePlan(planning_scene, req, res);

  if (res.error_code_ == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    time_param_.computeTimeStamps(*res.trajectory_, 1.0, 1.0);
  }

  moveit_msgs::MotionPlanResponse res_msg;
  res_msg.error_code = res.error_code_;
  res_msg.group_name = group_name;
  res_msg.trajectory_start = scene_msg.robot_state;
  res_msg.planning_time = res.planning_time_;

  if (res.error_code_ != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    return res_msg;
  }

  // add in all the other active joints in the robot, so that it's easier to use this trajectory in python/mujoco
  // where we don't have access to move groups.
  auto const &all_joints = robot_model->getActiveJointModelNames();
  res_msg.trajectory.joint_trajectory.joint_names = all_joints;
  for (auto i{0}; i < res.trajectory_->getWayPointCount(); ++i) {
    auto const &waypoint = res.trajectory_->getWayPoint(i);
    auto const &waypoint_names = waypoint.getVariableNames();
    trajectory_msgs::JointTrajectoryPoint point_msg;
    point_msg.time_from_start.fromSec(res.trajectory_->getWayPointDurationFromStart(i));
    // ensure positions matches the order of joint_names
    for (auto j{0}; j < all_joints.size(); ++j) {
      auto const &joint_name = all_joints[j];
      if (std::find(waypoint_names.cbegin(), waypoint_names.cend(), joint_name) != waypoint_names.cend()) {
        point_msg.positions.push_back(waypoint.getVariablePosition(joint_name));
        point_msg.velocities.push_back(waypoint.getVariableVelocity(joint_name));
        point_msg.accelerations.push_back(waypoint.getVariableAcceleration(joint_name));
      } else {
        point_msg.positions.push_back(0);
        point_msg.velocities.push_back(0);
        point_msg.accelerations.push_back(0);
      }
    }
    res_msg.trajectory.joint_trajectory.points.push_back(point_msg);
  }

  return res_msg;
}

void RRTPlanner::display_result(moveit_msgs::MotionPlanResponse const &res_msg) {
  moveit_msgs::DisplayTrajectory disp_res_msg;
  disp_res_msg.model_id = robot_model->getName();
  disp_res_msg.trajectory.push_back(res_msg.trajectory);
  disp_res_msg.trajectory_start = res_msg.trajectory_start;

  wait_for_connection(sln_traj_pub);
  sln_traj_pub.publish(disp_res_msg);
}