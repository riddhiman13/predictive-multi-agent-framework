// ROS
#include "ros/ros.h"
// C++
#include <mutex>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <thread>

#include "bimanual_planning_ros/vrep_controller.h"
#include "dqrobotics/DQ.h"
#include "dqrobotics/interfaces/vrep/DQ_VrepInterface.h"
#include "dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h"
#include "dqrobotics/robot_modeling/DQ_SerialManipulator.h"
#include "dqrobotics/utils/DQ_LinearAlgebra.h"

namespace ghostplanner {
namespace dqcontroller {
VrepController::VrepController(DQ_VrepInterface &vi,
    DQ_CooperativeDualTaskSpace &panda_bimanual,
    const std::vector<std::string> &joint_names_l,
    const std::vector<std::string> &joint_names_r, const std::string& robot_l,
    const std::string& robot_r, const double delta_t)
    : q_coop_curr_(VectorXd::Zero(14, 1)),
      dq_(VectorXd::Zero(14, 1)),
      q_l_curr_(VectorXd::Zero(7, 1)),
      q_r_curr_(VectorXd::Zero(7, 1)),
      vi_(vi),
      panda_bimanual_(panda_bimanual),
      joint_names_l_(joint_names_l),
      joint_names_r_(joint_names_r),
      robot_l_(robot_l),
      robot_r_(robot_r),
      delta_t_(delta_t) {
  Vector7d tmp;
  tmp << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  q_min_ << tmp, tmp;
  tmp << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  q_max_ << tmp, tmp;
}

void VrepController::controllerParamsCallback(
    const bimanual_planning_ros::ControllerParams &p) {
  std::vector<ControllerType> controller_type;
  std::vector<double> gains;
  for (int i = 0; i < p.gains.size(); ++i) {
    controller_type.push_back(ControllerType(p.controllers[i].data));
    gains.push_back(p.gains[i]);
  }
  ROS_INFO("velocity: %f", p.velocity);
  if (p.switching) {
    ROS_INFO("switching: true");
  } else {
    ROS_INFO("switching: false");
  }
  for (int i = 0; i < gains.size(); ++i) {
    ROS_INFO("gain[%i]: %12f, controller[%i]: %i", i, gains[i], i,
             controller_type[i]);
  }
  controller_.setTasks(controller_type, gains);
  controller_.setSwitching(p.switching);
  velocity_ = p.velocity;
}

void VrepController::addObstaclesLeftCallback(
    const bimanual_planning_ros::CollisionObjects& msg) {
  controller_.addObstaclesLeft(msg.objects);
}

void VrepController::addObstaclesRightCallback(
    const bimanual_planning_ros::CollisionObjects& msg) {
  controller_.addObstaclesRight(msg.objects);
}

void VrepController::moveObstaclesLeftCallback(
    const bimanual_planning_ros::CollisionObjects& msg) {
  controller_.moveObstaclesLeft(msg.objects);
}

void VrepController::moveObstaclesRightCallback(
    const bimanual_planning_ros::CollisionObjects& msg) {
  controller_.moveObstaclesRight(msg.objects);
}

void VrepController::targetPoseCallback(
    const bimanual_planning_ros::Position &msg) {
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
      ROS_WARN("target pose discarded as robot is in wrong state.");
      return;
    }
  }
  planned_pos_ = Vector3d(msg.data.data());
  if (!controller_.readyForNextPoint()) {
    ROS_WARN_STREAM("Discarding point: " << planned_pos_.transpose() <<
                    " as buffer is busy. Please wait until receiving a position"
                    " response before sending the next trajectory point.");
  }
  published_position_ = false;
  controller_.fillBuffer(planned_pos_);
  ros::Rate rate(1000);
  while (!published_position_ && ros::ok()) {
    update();
    rate.sleep();
  }
}

void VrepController::publishPosition() {
  Vector3d absolute_pos;
  DQ abp = panda_bimanual_.absolute_pose(q_coop_curr_);
  absolute_pos = abp.translation().q.segment(1, 3);
  bimanual_planning_ros::Position abs_pos;
  abs_pos.data = {absolute_pos.x(), absolute_pos.y(), absolute_pos.z()};
  pose_pub_.publish(abs_pos);
}

void VrepController::publishAbsPose() {
  DQ abp = panda_bimanual_.absolute_pose(q_coop_curr_);
  DQ delta = abp.P().conj() * DQ(cos(M_PI / 2), sin(M_PI / 2), 0.0, 0.0);
  DQ tray_pose = abp * delta;
  tray_pose = tray_pose * tray_pose.inv().norm();
  Vector3d y_l = panda_bimanual_.pose1(q_l_curr_).translation().q.segment(1, 3);
  y_l.z() = 0.0;
  Vector3d y_r = panda_bimanual_.pose2(q_r_curr_).translation().q.segment(1, 3);
  y_r.z() = 0.0;
  Vector3d line = y_r - y_l;
  double theta =
      acos((line / line.norm()).transpose() * Vector3d(0.0, 1.0, 0.0));
  if (line.x() > 0) {
    theta = 2 * M_PI - theta;
  }
  tray_pose = tray_pose * DQ(cos(theta / 2), 0.0, 0.0, sin(theta / 2));
  Vector3d absolute_pos;
  absolute_pos = tray_pose.translation().q.segment(1, 3);
  VectorXd q_temp = tray_pose.vec4();
  Quaterniond absolute_q;
  absolute_q.x() = q_temp.w();
  absolute_q.y() = q_temp.x();
  absolute_q.z() = q_temp.y();
  absolute_q.w() = q_temp.z();

  geometry_msgs::PoseStamped abs_pose;
  abs_pose.header.frame_id = "/base_link";
  abs_pose.header.stamp = ros::Time::now();
  abs_pose.pose.position.x = absolute_pos.x();
  abs_pose.pose.position.y = absolute_pos.y();
  abs_pose.pose.position.z = absolute_pos.z();
  abs_pose.pose.orientation.x = absolute_q.x();
  abs_pose.pose.orientation.y = absolute_q.y();
  abs_pose.pose.orientation.z = absolute_q.z();
  abs_pose.pose.orientation.w = absolute_q.w();

  abs_pose_pub_.publish(abs_pose);
}

void VrepController::publishJointStates() {
  sensor_msgs::JointState msg;
  msg.header.frame_id = robot_l_ + "_link0";
  msg.header.stamp = ros::Time::now();
  msg.name = joint_names_l_;
  msg.position.resize(q_l_curr_.size());
  VectorXd::Map(&msg.position[0], q_l_curr_.size()) = q_l_curr_;
  msg.position.push_back(0);
  msg.name.push_back(robot_l_ + "_finger_joint1");
  joint_l_pub_.publish(msg);

  msg.header.frame_id = robot_r_ + "_link0";
  msg.name = joint_names_r_;
  VectorXd::Map(&msg.position[0], q_r_curr_.size()) = q_r_curr_;
  msg.name.push_back(robot_r_ + "_finger_joint1");
  joint_r_pub_.publish(msg);
}

void VrepController::init(Vector3d r_B_O_r, Quaterniond R_B_O_r,
                          Vector3d r_B_O_l, Quaterniond R_B_O_l) {
  std::lock_guard<std::mutex> lock(control_mutex_);
  state_ = ControlState::IDLE;
  planner_sub_ =
      rosnode_.subscribe("goals", 1,
                         &VrepController::targetPoseCallback, this);
  params_sub_ = rosnode_.subscribe(
      "controller_params", 1,
      &VrepController::controllerParamsCallback, this);
  pose_pub_ = rosnode_.advertise<bimanual_planning_ros::Position>(
      "position", 1, true);
  abs_pose_pub_ =
      rosnode_.advertise<geometry_msgs::PoseStamped>("abs_pose", 1, true);
  start_obstacles_pub_ =
      rosnode_.advertise<std_msgs::Bool>("start_obstacles", 1, true);
  obstacle_add_sub_l_ = rosnode_.subscribe(
      "collision_objects_add_left", 1,
      &VrepController::addObstaclesLeftCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  obstacle_add_sub_r_ = rosnode_.subscribe(
      "collision_objects_add_right", 1,
      &VrepController::addObstaclesRightCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  obstacle_move_sub_l_ = rosnode_.subscribe(
      "collision_objects_move_left", 1,
      &VrepController::moveObstaclesLeftCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  obstacle_move_sub_r_ = rosnode_.subscribe(
      "collision_objects_move_right", 1,
      &VrepController::moveObstaclesRightCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  joint_l_pub_ = rosnode_.advertise<sensor_msgs::JointState>(robot_l_ + "/joint_states_vrep", 1);
  joint_r_pub_ = rosnode_.advertise<sensor_msgs::JointState>(robot_r_ + "/joint_states_vrep", 1);
  joint_motion_server_ = rosnode_.advertiseService("joint_motion",
      &VrepController::jointMotionCallback, this);
  controller_.init(r_B_O_r, R_B_O_r, r_B_O_l, R_B_O_l);
  controller_.setWholeBodyDistanceThresh(0.2);  // TODO param
  q_l_curr_ = vi_.get_joint_positions(joint_names_l_);
  q_r_curr_ = vi_.get_joint_positions(joint_names_r_);
  q_coop_curr_ << q_l_curr_, q_r_curr_;
  controller_.reset(q_coop_curr_);
}

void VrepController::starting() {
  std::lock_guard<std::mutex> lock(control_mutex_);
  state_ = ControlState::IDLE;
  q_l_curr_ = vi_.get_joint_positions(joint_names_l_);
  q_r_curr_ = vi_.get_joint_positions(joint_names_r_);
  q_coop_curr_ << q_l_curr_, q_r_curr_;
  controller_.reset(q_coop_curr_);
  controller_.calculateControlPreliminaries(q_coop_curr_);
}

void VrepController::initSwitchingControl() {
  std_msgs::Bool start;
  start.data = true;
  start_obstacles_pub_.publish(start);
  dq_.setZero();
  controller_.reset(q_coop_curr_);
}

bool VrepController::jointMotionCallback(
    bimanual_planning_ros::JointMotion::Request& req,
    bimanual_planning_ros::JointMotion::Response& res) {
  if (req.v>2) {
    ROS_WARN("Maximum speed of motion is 2 rad/s. Aborting");
    return false;
  }
  state_ = ControlState::JOINT_MOTION;
  Vector14d q_start, q, goal(req.goal.data());
  q_start << q_l_curr_, q_r_curr_;
  q = q_start;
  ros::Rate rate(1000);
  ROS_INFO_STREAM("Joint motion requested. Goal: " << goal.transpose());
  while (q!=goal && ros::ok()) {
    q += (goal-q_start)/(goal-q_start).array().abs().maxCoeff()*req.v*0.001;
    if ((q-q_start).dot(goal-q) < 0) {
      q = goal;
    }
    vi_.set_joint_positions(joint_names_l_, q.head(7));
    vi_.set_joint_positions(joint_names_r_, q.tail(7));
    q_l_curr_ = q.head(7);
    q_r_curr_ = q.tail(7);
    publishJointStates();
    rate.sleep();
  }
  q_coop_curr_ = goal;
  q_l_curr_ = q_coop_curr_.head(7);
  q_r_curr_ = q_coop_curr_.tail(7);
  controller_.calculateControlPreliminaries(q_coop_curr_);
  planned_pos_ = controller_.abs_pos();
  publishPosition();
  publishJointStates();
  publishAbsPose();
  state_ = ControlState::IDLE;
  return true;
}

void VrepController::update() {
  switch (state_) {
    case ControlState::INIT_SWITCHING_CONTROL: {
      initSwitchingControl();
      state_ = ControlState::SWITCHING_CONTROL;
    }
    case ControlState::SWITCHING_CONTROL: {
      // Calculate desired joint velocity
      Vector14d q_d_old = q_coop_curr_, qD_d_;
      q_coop_curr_ = controller_.followTrajectory(Vector14d::Zero(),
                                                  velocity_/0.9);
      q_l_curr_ = q_coop_curr_.head(7);
      q_r_curr_ = q_coop_curr_.tail(7);
      for (int i=0;i<14;++i) {
        if (q_coop_curr_[i] >= q_max_[i] || q_coop_curr_[i] <= q_min_[i]) {
          ROS_WARN_THROTTLE(1, "Robot is hitting joint limits in joint %d of %s"
                               "arm.", (i % 7) + 1, i >= 7 ? "right" : "left");
        }
      }
      qD_d_ = (q_coop_curr_ - q_d_old)*1000;
      break;
    }
    case ControlState::JOINT_MOTION: {
      break;
    }
    case ControlState::IDLE: {
      publishPosition();
      publishAbsPose();
      break;
    }
  }
  // Send desired values to VREP
  vi_.set_joint_positions(joint_names_l_, q_l_curr_);
  vi_.set_joint_positions(joint_names_r_, q_r_curr_);
  publishJointStates();
  if (controller_.readyForNextPoint() && !published_position_) {
    publishPosition();
    publishAbsPose();
    published_position_ = true;
  }
}

}  // namespace dqcontroller
}  // namespace ghostplanner
