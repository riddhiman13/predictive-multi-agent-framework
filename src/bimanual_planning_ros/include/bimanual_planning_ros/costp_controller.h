// Copyright (c) 2020 Lehrstuhl für Robotik und Sysstemintelligenz, TU München
#pragma once

#include <bimanual_planning_ros/controller_type.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

#include <atomic>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <bimanual_planning_ros/Obstacles.h>
#include <bimanual_planning_ros/trajectory_buffer.h>
#include <ros/ros.h>
#include <vector>
#include <utility>
#include "bimanual_planning_ros/cf_agent.h"
#include "bimanual_planning_ros/obstacle.h"
#include <bimanual_planning_ros/moveit_whole_body_avoidance.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 14, 1> Vector14d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

using DQ_robotics::DQ;
using DQ_robotics::DQ_CooperativeDualTaskSpace;
using DQ_robotics::DQ_Kinematics;
using DQ_robotics::DQ_SerialManipulator;
using ghostplanner::cfplanner::Obstacle;

namespace bimanual_planning_ros {

class CoSTPController {
 public:
  CoSTPController();
  ~CoSTPController() = default;
  void fillBuffer(const Vector3d& goal);
  Vector14d followTrajectory(const Vector14d& tau_ext, const double& vel);
  void init(Vector3d r_B_O_r, Quaterniond R_B_O_r, Vector3d r_B_O_l,
            Quaterniond R_B_O_l);
  void reset(const Vector14d& q);
  void addObstaclesLeft(const std::vector<moveit_msgs::CollisionObject>& msg);
  void addObstaclesRight(const std::vector<moveit_msgs::CollisionObject>& msg);
  void moveObstaclesLeft(const std::vector<moveit_msgs::CollisionObject>& msg);
  void moveObstaclesRight(const std::vector<moveit_msgs::CollisionObject>& msg);
  void setTasks(std::vector<ControllerType>& controller_type,
                std::vector<double>& gains);
  void setWholeBodyDistanceThresh(double distance_0);
  void setSwitching(bool switching);
  void calculateControlPreliminaries(const Vector14d& q);
  const Vector8d& dqrd() { return dqrd_.q; };
  const Vector8d& dqad() { return dqad_.q; };
  const Vector8d& rel_error() { return rel_error_; };
  const Vector3d& abs_error() { return abs_error_; };
  Vector3d rel_pos() { return dqrd_.translation().q.segment(1, 3); };
  Vector3d abs_pos() { return dqad_.translation().q.segment(1, 3); };
  const double& e_n() { return e_n_; };
  const double& dot_s() { return dot_s_; };
  const double& angle() { return angle_; };
  bool readyForNextPoint() { return ready_for_next_point_; };
 private:
  // control variables and intermediate results
  Eigen::Matrix<double, 8, 14> AbsPJ_;
  Eigen::Matrix<double, 8, 14> RelPJ_;
  Eigen::Matrix<double, 6, 14> geomJac_;
  Vector14d q_c_, dq_, qD_max_, q_min_, q_max_, q_, qD_;
  Vector8d rel_error_;
  Vector3d abs_error_, rot_err_int_, cg_, cg_buf_, lg_, next_ig_, last_ng_,
           current_ng_, goal_direction_ = Vector3d::Zero(), current_ig_;
  double next_ng_, v_act_, v_goal_, v_max_buf_;
  double e_n_, dot_s_, angle_, catchup_reserve_ = 0.1;
  DQ dqrd_, dqad_, l_, lz_, ee_orientation_1_, r_, relp_;
  bool switching_ = true;
  std::atomic<bool> ready_for_next_point_ = false;
  // body agents for collision avoidance (joints that will avoid collisions)
  // TODO this should be initialized from yaml file
  std::vector<std::pair<int, Eigen::Vector3d>> avoidance_poi_{ {4, {0, 0, 0}} };
  ghostplanner::cfplanner::CfAgent body_avoidance_agent_;
  // DQ dual Panda representation
  std::unique_ptr<DQ_SerialManipulator> franka_left_;
  std::unique_ptr<DQ_SerialManipulator> franka_right_;
  std::unique_ptr<DQ_CooperativeDualTaskSpace> dq_dual_panda_;
  // Controller configuration
  std::vector<ControllerType> controller_type_;
  std::unordered_map<ControllerType::Type, double> gains_;
  std::unique_ptr<TrajectoryBuffer> tb_;
  std::recursive_mutex traj_buf_mutex_;
  //MoveitWholeBodyAvoidance mwba_;
  // control functions and tasks
  Vector14d control(const Vector14d& q, const Vector14d& qD,
      Vector3d current_goal, const Vector14d& tau_ext, const double& vel);
  void positionControlImpl(MatrixXd& Jac, Vector14d& err, const Vector3d& cg,
      const Vector3d& x_ff, const ControllerType::Type& ct);
  Vector3d getInstantaneousGoal();
  void getCurrentNominalGoal();
  void relativePoseControl(MatrixXd& Jac, Vector14d& err);
  void absolutePositionControl(MatrixXd& Jac, Vector14d& err,
                               const Vector3d& cg, const double& vel);
  void EETiltControl(MatrixXd& Jac, Vector14d& err);
  void rotationalAdmittanceControl(MatrixXd& Jac, Vector14d& err,
      const Vector14d& tau_ext);
  void jointLimitAvoidanceControl(MatrixXd& Jac, Vector14d& err,
                                  const Vector14d& q);
  void wholeBodyCollisionAvoidance(MatrixXd& Jac, Vector14d& err,
                                   const Vector14d& q);
  void switchingOrder(const Vector14d& q, const Vector14d& qD);
  bool tiltFunc(const double& absangle, const double& tilt_min,
                const double& tilt_max) const;
  bool boolFunc(const Vector14d& q, const Vector14d& qD) const;
  bool boolJoint(const Vector14d& q, std::vector<int>& joints) const;
};
MatrixXd geomJ(const MatrixXd& absoluteposeJ, const DQ& absolutepose);
MatrixXd geomJ(const DQ_SerialManipulator &robot, const MatrixXd &poseJacobian,
               const VectorXd &q, const int n);
}  // namespace bimanual_planning_ros
