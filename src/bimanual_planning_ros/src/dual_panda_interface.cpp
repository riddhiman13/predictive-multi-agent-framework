#include <ros/ros.h>

#include <bimanual_planning_ros/dual_panda_interface.h>
#include <franka_msgs/SetForceTorqueCollisionBehavior.h>

DualPandaInterface::DualPandaInterface() :
    gc_l_("left/franka_gripper/grasp", true),
    rc_l_("left/franka_gripper/move", true),
    gc_r_("right/franka_gripper/grasp", true),
    rc_r_("right/franka_gripper/move", true) {};

void DualPandaInterface::init() {
  ROS_INFO("Waiting for gripper clients...");
  gc_l_.waitForServer();
  rc_l_.waitForServer();
  gc_r_.waitForServer();
  rc_r_.waitForServer();
  ROS_INFO("Done.");
}

void DualPandaInterface::openGripper(const bool& left, const double& width) {
  franka_gripper::MoveGoal goal;
  if (width <= 0.07) {
    goal.width = width;
  } else {
    ROS_WARN("Width greater 0.07 ignored and replaced by max of 0.07");
    goal.width = 0.07;
  }
  goal.speed = 0.05;
  if (left) {
    rc_l_.sendGoal(goal);
    rc_l_.waitForResult();
  } else {
    rc_r_.sendGoal(goal);
    rc_r_.waitForResult();
  }
}

void DualPandaInterface::openGrippers(const double& width) {
  franka_gripper::MoveGoal goal;
  if (width <= 0.07) {
    goal.width = width;
  } else {
    ROS_WARN("Width greater 0.07 ignored and replaced by max of 0.07");
    goal.width = 0.07;
  }
  goal.speed = 0.05;
  rc_l_.sendGoal(goal);
  rc_r_.sendGoal(goal);
  rc_l_.waitForResult();
  rc_r_.waitForResult();
}

void DualPandaInterface::closeGripper(const bool& left) {
  franka_gripper::GraspGoal goal;
  goal.width = 0.0;
  goal.speed = 0.05;
  goal.epsilon.inner = 1;
  goal.epsilon.outer = 1;
  goal.force = 70;
  if (left) {
    gc_l_.sendGoal(goal);
    gc_l_.waitForResult();
  } else {
    gc_r_.sendGoal(goal);
    gc_r_.waitForResult();
  }
}

void DualPandaInterface::closeGrippers() {
  franka_gripper::GraspGoal goal;
  goal.width = 0.0;
  goal.speed = 0.05;
  goal.epsilon.inner = 1;
  goal.epsilon.outer = 1;
  goal.force = 70;
  gc_l_.sendGoal(goal);
  gc_r_.sendGoal(goal);
  gc_l_.waitForResult();
  gc_r_.waitForResult();
}

void DualPandaInterface::setReflexThresholds(const  std::vector<double>& F_max,
    const std::vector<double>& tau_max) {
  franka_msgs::SetForceTorqueCollisionBehavior srv;
  for (int i = 0; i < 6; ++i) {
    srv.request.lower_force_thresholds_nominal[i] = F_max[i];
    srv.request.upper_force_thresholds_nominal[i] = F_max[i];
  }
  for (int i = 0; i < 7; ++i) {
    srv.request.lower_torque_thresholds_nominal[i] = tau_max[i];
    srv.request.upper_torque_thresholds_nominal[i] = tau_max[i];
  }
  if (!ros::service::waitForService(
          "left/set_force_torque_collision_behavior", 10) ||
      !ros::service::waitForService(
          "right/set_force_torque_collision_behavior", 10))
  {
    ROS_ERROR("Error. Service to set collision behavior did not come up.");
    throw "Threshold error";
  }
  if (!ros::service::call(
          "left/set_force_torque_collision_behavior", srv))
  {
    ROS_ERROR("Error: SetCollisionBehaviorService for left arm failed.");
    throw "Threshold error";
  }
  else if (!srv.response.success)
  {
    ROS_ERROR("Error setting left arm collision behavior: %s.",
              srv.response.error.c_str());
    throw "Threshold error";
  }
  if (!ros::service::call(
          "right/set_force_torque_collision_behavior", srv))
  {
    ROS_ERROR("Error: SetCollisionBehaviorService for right arm failed.");
    throw "Threshold error";
  }
  else if (!srv.response.success)
  {
    ROS_ERROR("Error setting right arm collision behavior: %s.",
              srv.response.error.c_str());
    throw "Threshold error";
  }
}
