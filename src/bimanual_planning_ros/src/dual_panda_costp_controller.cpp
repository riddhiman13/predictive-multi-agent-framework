// Copyright (c) 2020 Lehrstuhl für Robotik und Sysstemintelligenz, TU München
#include <bimanual_planning_ros/dual_panda_costp_controller.h>

#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/trigger_rate.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <bimanual_planning_ros/ContactWrench.h>
#include <bimanual_planning_ros/franka_robot.h>
#include <bimanual_planning_ros/joint_motion_generator.h>
#include <bimanual_planning_ros/panda_joint_impedance_controller.h>

namespace bimanual_planning_ros {

Vector3d DualPandaCoSTPController::B_T_O_l(Vector3d p_0) {
  return std::move(Vector3d(r_B_O_l_+B_R_O_l_*p_0));
}

Vector3d DualPandaCoSTPController::B_T_O_r(Vector3d p_0) {
  return std::move(Vector3d(r_B_O_r_+B_R_O_r_*p_0));
}

Vector3d DualPandaCoSTPController::O_T_B_l(Vector3d p_B) {
  return std::move(Vector3d(O_R_B_l_*(p_B-r_B_O_l_)));
}

Vector3d DualPandaCoSTPController::O_T_B_r(Vector3d p_B) {
  return std::move(Vector3d(B_R_O_r_*(p_B-r_B_O_r_)));
}

Quaterniond DualPandaCoSTPController::B_T_O_l(Quaterniond q_0) {
  return std::move(Quaterniond(B_R_O_l_*q_0.toRotationMatrix()));
}

Quaterniond DualPandaCoSTPController::B_T_O_r(Quaterniond q_0) {
  return std::move(Quaterniond(B_R_O_r_*q_0.toRotationMatrix()));
}

Quaterniond DualPandaCoSTPController::O_T_B_l(Quaterniond q_B) {
  return std::move(Quaterniond(O_R_B_l_*q_B.toRotationMatrix()));
}

Quaterniond DualPandaCoSTPController::O_T_B_r(Quaterniond q_B) {
  return std::move(Quaterniond(O_R_B_r_*q_B.toRotationMatrix()));
}

DualPandaCoSTPController::DualPandaCoSTPController() :
    publish_rate_(100) {}

bool DualPandaCoSTPController::init(hardware_interface::RobotHW* robot_hw,
                                    ros::NodeHandle& node_handle) {
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
  if (!node_handle.getParam("planning_frame", planning_frame_)) {
    ROS_ERROR_STREAM(
        "DualPandaCoSTPController: Could not read parameter planning_frame");
    return false;
  }
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  std::string error;
  if (!tfBuffer.canTransform(planning_frame_, right_arm_id + "_link0",
                             ros::Time(0), ros::Duration(3), &error)) {
    ROS_ERROR("DualPandaCoSTPController: Could transform from planning_frame "
              "%s to arm frame %s. Error: %s", planning_frame_.c_str(),
              (right_arm_id + "_link0").c_str(), error.c_str());
  }
  if (!tfBuffer.canTransform(planning_frame_, left_arm_id + "_link0",
                             ros::Time(0), ros::Duration(3), &error)) {
    ROS_ERROR("DualPandaCoSTPController: Could transform from planning_frame "
              "%s to arm frame %s. Error: %s", planning_frame_.c_str(),
              (left_arm_id + "_link0").c_str(), error.c_str());
  }
  geometry_msgs::TransformStamped transform;
  transform = tfBuffer.lookupTransform(planning_frame_, right_arm_id + "_link0",
                                       ros::Time(0), ros::Duration(3));
  r_B_O_r_ = Vector3d(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
  Quaterniond B_R_O_r(transform.transform.rotation.w,
                      transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z);
  B_R_O_r_ = B_R_O_r.toRotationMatrix();
  O_R_B_r_ = B_R_O_r_.transpose();
  transform = tfBuffer.lookupTransform(planning_frame_, left_arm_id + "_link0",
                                       ros::Time(0), ros::Duration(3));
  r_B_O_l_ = Vector3d(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
  Quaterniond B_R_O_l(transform.transform.rotation.w,
                      transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z);
  controller_.setWholeBodyDistanceThresh(0.2);
  controller_.init(r_B_O_r_, B_R_O_r, r_B_O_l_, B_R_O_l);
  B_R_O_l_ = B_R_O_l.toRotationMatrix();
  O_R_B_l_ = B_R_O_l_.transpose();
  sub_target_pose_ = node_handle.subscribe("goals", 1,
      &DualPandaCoSTPController::targetPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_params_ = node_handle.subscribe("controller_params", 1,
      &DualPandaCoSTPController::controllerParamsCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  info_pub_ = node_handle.advertise<ControllerInfo>("info", 1);
  contact_wrench_pub_ = node_handle.advertise<ContactWrench>(
      "contact_wrench", 1);
  joint_motion_server_ = node_handle.advertiseService("joint_motion",
      &DualPandaCoSTPController::jointMotionCallback, this);
  obstacle_add_sub_l_ = node_handle.subscribe(
      "collision_objects_add_left", 1,
      &DualPandaCoSTPController::addObstaclesLeftCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  obstacle_add_sub_r_ = node_handle.subscribe(
      "collision_objects_add_right", 1,
      &DualPandaCoSTPController::addObstaclesRightCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  obstacle_move_sub_l_ = node_handle.subscribe(
      "collision_objects_move_left", 1,
      &DualPandaCoSTPController::moveObstaclesLeftCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  obstacle_move_sub_r_ = node_handle.subscribe(
      "collision_objects_move_right", 1,
      &DualPandaCoSTPController::moveObstaclesRightCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  bool left_success = left_controller_.init(robot_hw, left_arm_id,
      left_joint_names);
  bool right_success = right_controller_.init(robot_hw, right_arm_id,
      right_joint_names);
  // Setup publisher for the centering frame.
  p_pub_ = node_handle.advertise<Position>("position", 1, true);
  return left_success && right_success;
}

void DualPandaCoSTPController::starting(const ros::Time& /*time*/) {
  std::lock_guard<std::mutex> lock(control_mutex_);
  state_ = ControlState::IDLE;
  Vector7d q_l(left_controller_.getQ()),
           q_r(right_controller_.getQ());
  q_d_ << q_l, q_r;
  controller_.reset(q_d_);
}

Vector6d transformWrenchFromOToB(const franka::RobotState& state,
    const Eigen::Matrix3d& B_R_O) {
  Eigen::Matrix<double, 3, 1> O_f_ext_K;
  Eigen::Matrix<double, 3, 1> O_m_ext_K;
  Eigen::Matrix<double, 4, 4> O_T_EE(state.O_T_EE.data());
  Eigen::Matrix<double, 4, 4> EE_T_K(state.EE_T_K.data());
  Eigen::Matrix<double, 4, 4> O_T_K = O_T_EE * EE_T_K;
  Eigen::Matrix<double, 3, 1> r_OK = O_T_K.topRows(3).col(3);
  for (int i=0; i<3; ++i) {
    O_f_ext_K(i) = state.O_F_ext_hat_K[i];
    O_m_ext_K(i) = state.O_F_ext_hat_K[i+3];
  }
  Eigen::Matrix<double, 6, 1> B_F_ext_K;
  B_F_ext_K.topRows(3) = B_R_O * O_f_ext_K;
  B_F_ext_K.bottomRows(3) = B_R_O * (O_m_ext_K - r_OK.cross(O_f_ext_K));
  return B_F_ext_K;
}

void DualPandaCoSTPController::initSwitchingControl() {
  qD_d_.setZero();
  controller_.reset(q_d_);
}

void DualPandaCoSTPController::computeContactWrench() {
  Vector6d F_l = transformWrenchFromOToB(left_controller_.getRobotState(),
                                         B_R_O_l_);
  Vector6d F_r = transformWrenchFromOToB(right_controller_.getRobotState(),
                                         B_R_O_r_);
  Vector3d d = controller_.rel_pos();
  contact_wrench_.head(3) = F_l.head(3) + F_r.head(3);
  contact_wrench_.tail(3) = F_l.tail(3) + F_r.tail(3) -
      0.5*d.cross(Vector3d(F_l.head(3))) + 0.5*d.cross(Vector3d(F_r.head(3)));
}

void DualPandaCoSTPController::printErrors() {
  franka::Errors ce_l = left_controller_.getRobotState().current_errors;
  franka::Errors ce_r = right_controller_.getRobotState().current_errors;
  static franka::Errors le_l = ce_l;
  static franka::Errors le_r = ce_r;
  if (std::string(ce_l).compare(std::string(le_l))) {
    ROS_ERROR_STREAM("Left arm has errors: " << std::string(ce_l));
  }
  if (std::string(ce_r).compare(std::string(le_r))) {
    ROS_ERROR_STREAM("Right arm has errors: " << std::string(ce_r));
  }
  le_l = ce_l;
  le_r = ce_r;
}

void DualPandaCoSTPController::update(const ros::Time& /*time*/,
                                      const ros::Duration& /*period*/) {
  static const double t = 0.001;
  std::lock_guard<std::mutex> lock(control_mutex_);
  computeContactWrench();
  switch (state_) {
    case ControlState::INIT_SWITCHING_CONTROL: {
      initSwitchingControl();
      state_ = ControlState::SWITCHING_CONTROL;
    }
    case ControlState::SWITCHING_CONTROL: {
      if (new_point_) {
        controller_.fillBuffer(current_goal_);
        new_point_ = false;
      }
      Vector14d q, qD, q_d_old, tau_ext;
      Vector7d q_l(left_controller_.getQ()),
               q_r(right_controller_.getQ());
      Vector7d qD_l(left_controller_.getQD()),
               qD_r(right_controller_.getQD());
      Vector7d tau_ext_l(
          left_controller_.getRobotState().tau_ext_hat_filtered.data()),
          tau_ext_r(
          right_controller_.getRobotState().tau_ext_hat_filtered.data());
      q << q_l, q_r;
      qD << qD_l, qD_r;
      tau_ext << tau_ext_l, tau_ext_r;
      q_d_old = q_d_;
      q_d_ = controller_.followTrajectory(tau_ext, velocity_/0.9);
      qD_d_ = (q_d_ - q_d_old)*1000;
      break;
    }
    case ControlState::JOINT_MOTION:
    case ControlState::IDLE: {
      Vector14d q;
      Vector7d q_l(left_controller_.getQ()),
               q_r(right_controller_.getQ());
      q << q_l, q_r;
      controller_.calculateControlPreliminaries(q);
      break;
    }
  }
  left_controller_.control(q_d_.head(7), Vector7d::Zero());
  right_controller_.control(q_d_.tail(7), Vector7d::Zero());
  if (publish_rate_()) {
    publishInfoError();
  }
  if (controller_.readyForNextPoint() && !published_position_) {
    publishPosition();
    published_position_ = true;
  }
  publishContactWrench();
  printErrors();
}

void DualPandaCoSTPController::controllerParamsCallback(
    const ControllerParams& p) {
  std::vector<ControllerType> controller_type;
  std::vector<double> gains;
  for (int i=0;i<p.gains.size();++i) {
    controller_type.push_back(ControllerType(p.controllers[i].data));
    gains.push_back(p.gains[i]);
  }
  for (int i=0;i<gains.size();++i) {
    ROS_INFO("gain[%i]: %12f, controller[%i]: %i", i, gains[i], i,
             controller_type[i]);
  }
  ROS_INFO("velocity: %f", p.velocity);
  if (p.switching) {
    ROS_INFO("switching: true");
  } else {
    ROS_INFO("switching: false");
  }
  double low_level_gain;
  if (p.low_level_gain < 0.0) {
    ROS_WARN("Low_level_gain has to be non-negative. Using value 0.0.");
    low_level_gain = 0;
  } else if (p.low_level_gain > 2.0) {
    ROS_WARN("Low_level_gain has to smaller than 2. Using value 2.0.");
    low_level_gain = 2;
  } else {
    low_level_gain = p.low_level_gain;
  }
  ROS_INFO("low_level_gain: %f", p.low_level_gain);
  std::lock_guard<std::mutex> lock(control_mutex_);
  controller_.setTasks(controller_type, gains);
  controller_.setSwitching(p.switching);
  velocity_ = p.velocity;
  left_controller_.setStiffnessScale(low_level_gain);
  right_controller_.setStiffnessScale(low_level_gain);
}

void DualPandaCoSTPController::addObstaclesLeftCallback(
    const CollisionObjects& msg) {
  controller_.addObstaclesLeft(msg.objects);
}

void DualPandaCoSTPController::addObstaclesRightCallback(
    const CollisionObjects& msg) {
  controller_.addObstaclesRight(msg.objects);
}

void DualPandaCoSTPController::moveObstaclesLeftCallback(
    const CollisionObjects& msg) {
  controller_.moveObstaclesLeft(msg.objects);
}

void DualPandaCoSTPController::moveObstaclesRightCallback(
    const CollisionObjects& msg) {
  controller_.moveObstaclesRight(msg.objects);
}

void DualPandaCoSTPController::targetPoseCallback(
    const Position& msg) {
  std::lock_guard<std::mutex> lock(control_mutex_);
  switch (state_) {
    case ControlState::IDLE: {
      state_ = ControlState::INIT_SWITCHING_CONTROL;
      break;
    }
    case ControlState::INIT_SWITCHING_CONTROL:
    case ControlState::SWITCHING_CONTROL: {
      break;
    }
    default: {
      ROS_WARN_THROTTLE(1, "target pose discarded as robot is in wrong state.");
      return;
    }
  }
  current_goal_ = Vector3d(msg.data.data());
  if (!controller_.readyForNextPoint()) {
    ROS_WARN_STREAM("Discarding point: " << current_goal_.transpose() <<
                    " as buffer is busy. Please wait until receiving a position"
                    " response before sending the next trajectory point.");
  }
  new_point_ = true;
  published_position_ = false;
}

bool DualPandaCoSTPController::jointMotionCallback(JointMotion::Request& req,
    JointMotion::Response& res) {
  if (req.v>2) {
    ROS_WARN("Maximum speed of motion is 2 rad/s. Aborting");
    return false;
  }
  Vector14d q, goal(req.goal.data());
  ROS_INFO_STREAM("Joint motion requested. Goal: " << goal.transpose());
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    Vector7d q_l(left_controller_.getQ()),
             q_r(right_controller_.getQ());
    q << q_l, q_r;
    state_ = ControlState::JOINT_MOTION;
    controller_.reset(q);
  }
  JointMotionGenerator::generateC1Trajectory(control_mutex_, q_d_, qD_d_, req.v,
                                             q, goal);
  std::lock_guard<std::mutex> lock(control_mutex_);
  state_ = ControlState::IDLE;
  publishPosition();
  return true;
}

void DualPandaCoSTPController::publishInfoError() {
  ControllerInfo ci;
  for (int i=0;i<8;++i) {
    ci.dqrd[i] = controller_.dqrd()[i];
    ci.dqad[i] = controller_.dqad()[i];
  }
  ci.rel_error = controller_.rel_error().norm();
  ci.abs_error = controller_.abs_error().norm();
  ci.e_n = controller_.e_n();
  ci.angle = controller_.angle();
  ci.dot_s = controller_.dot_s();
  for (int i=0;i<14;++i) {
    ci.dq[i] = dq_[i];
  }
  info_pub_.try_publish(ci);
}

void DualPandaCoSTPController::publishPosition() {
  Position msg;
  Vector3d abs_pos = controller_.abs_pos();
  msg.data = {abs_pos[0], abs_pos[1], abs_pos[2]};
  p_pub_.try_publish(msg);
}

void DualPandaCoSTPController::publishContactWrench() {
  Vector6d F = contact_wrench_;
  ContactWrench msg;
  for (int i=0;i<6;++i) {
    msg.B_F_ext[i] = contact_wrench_[i];
  }
  contact_wrench_pub_.try_publish(msg);
}

}  // namespace bimanual_planning_ros

PLUGINLIB_EXPORT_CLASS(bimanual_planning_ros::DualPandaCoSTPController,
                       controller_interface::ControllerBase)
