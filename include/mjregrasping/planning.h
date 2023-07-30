#pragma once

#include <moveit_msgs/RobotTrajectory.h>

#include <arc_utilities/eigen_typedefs.hpp>
#include <string>
#include <utility>
#include <vector>

using namespace EigenHelpers;

class RRTPlanner {
 public:
  RRTPlanner();

  std::pair<bool, moveit_msgs::RobotTrajectory> plan_from_q_to_target_pos(std::vector<double> const &q0,
                                                                          VectorVector3d const &candidate_pos,
                                                                          std::vector<std::string> const &strategy_strs,
                                                                          std::vector<bool> const &is_grasping0);
};