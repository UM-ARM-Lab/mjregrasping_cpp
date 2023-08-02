#include <moveit/robot_state/conversions.h>

#include <mjregrasping/planning.h>

namespace rvt = rviz_visual_tools;


int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_demo");

  std::string const group_name = "left_arm";

  auto planner = RRTPlanner();

  robot_state::RobotState state(planner.robot_model);
  state.setToDefaultValues();
  state.setVariablePosition("joint56", -0.4);
  state.setVariablePosition("joint57", 0.165);
  state.setVariablePosition("joint41", 0.4);

  moveit_msgs::PlanningScene scene_msg;
  scene_msg.is_diff = true;  // this needs to be a diff, otherwise we'll override the ACM which we don't want
  scene_msg.robot_state.is_diff = false;
  moveit::core::robotStateToRobotStateMsg(state, scene_msg.robot_state);
  scene_msg.world.collision_objects.resize(1);
  auto &co = scene_msg.world.collision_objects[0];
  co.id = "box1";
  co.header.frame_id = "robot_root";
  co.pose.orientation.w = 0.7071;
  co.pose.orientation.x = 0.0;
  co.pose.orientation.y = 0.0;
  co.pose.orientation.z = 0.7071;
  co.pose.position.x = -0.2;
  co.pose.position.y = 0.76;
  co.pose.position.z = 0.155;
  co.primitive_poses.resize(1);
  auto &prim_pose = co.primitive_poses[0];
  prim_pose.orientation.w = 1.0;
  co.primitives.resize(1);
  auto &prim_shape = co.primitives[0];
  prim_shape.dimensions.resize(3);
  prim_shape.dimensions[0] = 0.08;
  prim_shape.dimensions[1] = 0.28;
  prim_shape.dimensions[2] = 0.2;
  prim_shape.type = shape_msgs::SolidPrimitive::BOX;

  std::map<std::string, Eigen::Vector3d> goal_positions{
      //      {"left_tool", Eigen::Vector3d(-0.2, 0.8, 0.7)},
      {"left_tool", Eigen::Vector3d(-0.12, 0.862, 0.164)},
  };

  auto const res_msg = planner.plan(scene_msg, group_name, goal_positions, true);
  planner.display_result(res_msg);

  return 0;
}