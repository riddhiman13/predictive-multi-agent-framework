#pragma once
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <bimanual_planning_ros/robot_interface.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

class DualPandaInterface : public RobotInterface {
 public:
  DualPandaInterface();
  void init() override;
  void openGripper(const bool& left, const double& width) override;
  void openGrippers(const double& width) override;
  void closeGripper(const bool& left) override;
  void closeGrippers() override;
  void setReflexThresholds(const std::vector<double>& F_max,
                           const std::vector<double>& tau_max) override;
 private:
  actionlib::SimpleActionClient<franka_gripper::GraspAction> gc_l_;
  actionlib::SimpleActionClient<franka_gripper::GraspAction> gc_r_;
  actionlib::SimpleActionClient<franka_gripper::MoveAction> rc_l_;
  actionlib::SimpleActionClient<franka_gripper::MoveAction> rc_r_;
};
