// ROS
#include "ros/ros.h"
// C++
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <thread>
// Libraries
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <tf2_ros/static_transform_broadcaster.h>
// Current package
#include <bimanual_planning_ros/franka_robot.h>
#include "bimanual_planning_ros/costp_controller.h"
#include "bimanual_planning_ros/vrep_controller.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "vrep_interface");
  ros::NodeHandle node_handle;
  DQ_VrepInterface vi;
  vi.connect(19997, 100, 10);
  vi.start_simulation();
  std::string left_arm_id, right_arm_id;
  if (!node_handle.getParam("left/arm_id", left_arm_id)) {
    ROS_ERROR_STREAM(
        "DualPandaCoSTPController: Could not read parameter left_arm_id");
    return false;
  }
  std::vector<std::string> left_joint_names;
  if (!node_handle.getParam("left/joint_names", left_joint_names) ||
      left_joint_names.size() != 7) {
    ROS_ERROR(
        "DualPandaCoSTPController: Invalid or no left_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }
  if (!node_handle.getParam("right/arm_id", right_arm_id)) {
    ROS_ERROR_STREAM(
        "DualPandaCoSTPController: Could not read parameter right_arm_id");
    return false;
  }
  std::vector<std::string> right_joint_names;
  if (!node_handle.getParam("right/joint_names", right_joint_names) ||
      right_joint_names.size() != 7) {
    ROS_ERROR(
        "DualPandaCoSTPController: Invalid or no right_joint_names parameters"
        " provided, aborting controller init!");
    return false;
  }
  DQ pose_l = vi.get_object_pose(left_arm_id + "_link1");
  DQ pose_r = vi.get_object_pose(right_arm_id + "_link1");
  Vector3d r_B_O_l = pose_l.translation().q.segment(1, 3);
  Vector3d r_B_O_r = pose_r.translation().q.segment(1, 3);
  Eigen::Quaterniond R_B_O_l(pose_l.rotation().vec4()[0],
      pose_l.rotation().vec4()[1], pose_l.rotation().vec4()[2],
      pose_l.rotation().vec4()[3]);
  Eigen::Quaterniond R_B_O_r(pose_r.rotation().vec4()[0],
      pose_r.rotation().vec4()[1], pose_r.rotation().vec4()[2],
      pose_r.rotation().vec4()[3]);
  DQ p1 = pose_l.translation();
  DQ r1 = pose_l.rotation();
  r1 = r1 * r1.inv().norm();
  DQ_SerialManipulator robot_l =
      Panda::kinematics(r_B_O_l.data(), R_B_O_l.coeffs().data());
  robot_l.set_base_frame(r1 + 0.5 * E_ * p1 * r1);
  robot_l.set_reference_frame(r1 + 0.5 * E_ * p1 * r1);
  DQ p2 = pose_r.translation();
  DQ r2 = pose_r.rotation();
  r2 = r2 * r2.inv().norm();
  DQ_SerialManipulator robot_r =
      Panda::kinematics(r_B_O_r.data(), R_B_O_r.coeffs().data());
  robot_r.set_base_frame(r2 + 0.5 * E_ * p2 * r2);
  robot_r.set_reference_frame(r2 + 0.5 * E_ * p2 * r2);
  DQ_CooperativeDualTaskSpace panda_bimanual(&robot_l, &robot_r);
  double delta_t = 0.001;
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "base_link";
  static_transformStamped.child_frame_id = left_arm_id + "_link0";
  static_transformStamped.transform.translation.x = r_B_O_l[0];
  static_transformStamped.transform.translation.y = r_B_O_l[1];
  static_transformStamped.transform.translation.z = r_B_O_l[2];
  static_transformStamped.transform.rotation.x = R_B_O_l.x();
  static_transformStamped.transform.rotation.y = R_B_O_l.y();
  static_transformStamped.transform.rotation.z = R_B_O_l.z();
  static_transformStamped.transform.rotation.w = R_B_O_l.w();
  static_broadcaster.sendTransform(static_transformStamped);
  static_transformStamped.child_frame_id = right_arm_id + "_link0";
  static_transformStamped.transform.translation.x = r_B_O_r[0];
  static_transformStamped.transform.translation.y = r_B_O_r[1];
  static_transformStamped.transform.translation.z = r_B_O_r[2];
  static_transformStamped.transform.rotation.x = R_B_O_r.x();
  static_transformStamped.transform.rotation.y = R_B_O_r.y();
  static_transformStamped.transform.rotation.z = R_B_O_r.z();
  static_transformStamped.transform.rotation.w = R_B_O_r.w();
  static_broadcaster.sendTransform(static_transformStamped);
  ghostplanner::dqcontroller::VrepController VrepController(vi, panda_bimanual,
      left_joint_names, right_joint_names, left_arm_id, right_arm_id, delta_t);
  VrepController.init(r_B_O_r, R_B_O_r, r_B_O_l, R_B_O_l);
  for (int i=0;i<10;++i) {
    VrepController.update();
  }
  ros::spin();
  vi.stop_simulation();
  vi.disconnect();
  return 0;
}
