// Copyright (c) 2020 Lehrstuhl für Robotik und Sysstemintelligenz, TU München
#include <bimanual_planning_ros/costp_controller.h>
#include <bimanual_planning_ros/franka_robot.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <vector>
#include <algorithm>

using DQ_robotics::C4;
using DQ_robotics::DQ_SerialManipulator;
using DQ_robotics::E_;
using DQ_robotics::i_;
using DQ_robotics::j_;
using DQ_robotics::k_;
using DQ_robotics::Panda;
using DQ_robotics::pi;
using DQ_robotics::pinv;

namespace bimanual_planning_ros {

CoSTPController::CoSTPController() {
  tb_ = std::make_unique<TrajectoryBuffer>(1);
}

void CoSTPController::addObstaclesLeft(
    const std::vector<moveit_msgs::CollisionObject>& msg) {
  //mwba_.addObstaclesLeft(msg);
}

void CoSTPController::addObstaclesRight(
    const std::vector<moveit_msgs::CollisionObject>& msg) {
  //mwba_.addObstaclesRight(msg);
}

void CoSTPController::init(Vector3d r_B_O_r, Quaterniond B_R_O_r, Vector3d r_B_O_l, Quaterniond B_R_O_l)
{
  Vector7d tmp;
  tmp << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  q_min_ << tmp, tmp;
  tmp << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  q_max_ << tmp, tmp;
  q_c_ = 0.5 * (q_max_ + q_min_);
  franka_right_ = std::make_unique<DQ_SerialManipulator>(
      Panda::kinematics(r_B_O_r.data(), B_R_O_r.coeffs().data()));
  franka_left_ = std::make_unique<DQ_SerialManipulator>(
      Panda::kinematics(r_B_O_l.data(), B_R_O_l.coeffs().data()));
  DQ_Kinematics* fep1 = franka_left_.get();
  DQ_Kinematics* fep2 = franka_right_.get();
  dq_dual_panda_ = std::make_unique<DQ_CooperativeDualTaskSpace>(fep1, fep2);

  Eigen::Isometry3d base_tf_l, base_tf_r;
  base_tf_l.translation() = -B_R_O_l.toRotationMatrix().transpose()*r_B_O_l;
  base_tf_l.linear() = B_R_O_l.toRotationMatrix().transpose();
  base_tf_r.translation() = -B_R_O_r.toRotationMatrix().transpose()*r_B_O_r;
  base_tf_r.linear() = B_R_O_r.toRotationMatrix().transpose();
  //mwba_.init(base_tf_l, base_tf_r);
}

void CoSTPController::moveObstaclesLeft(
    const std::vector<moveit_msgs::CollisionObject>& msg) {
  //mwba_.moveObstaclesLeft(msg);
}

void CoSTPController::moveObstaclesRight(
    const std::vector<moveit_msgs::CollisionObject>& msg) {
  //mwba_.moveObstaclesRight(msg);
}

void CoSTPController::setTasks(std::vector<ControllerType>& controller_type,
                               std::vector<double>& gains) {
  for (size_t i = 0; i < gains.size(); i++) {
    gains_[controller_type[i]] = gains[i];
  }
  controller_type_ = controller_type;
}

void CoSTPController::setSwitching(bool switching) {
  switching_ = switching;
}

void CoSTPController::setWholeBodyDistanceThresh(double distance_0) {
  //mwba_.setWholeBodyDistanceThresh(distance_0);
}

void CoSTPController::reset(const Vector14d& q) {
  ready_for_next_point_ = true;
  dqad_ = dq_dual_panda_->absolute_pose(q);
  dqrd_ = dq_dual_panda_->relative_pose(q);
  // Desired Plücker line in k_
  ee_orientation_1_ = dqad_.P();
  lz_ = ee_orientation_1_.conj() * k_ * ee_orientation_1_;
  rot_err_int_.setZero();
  tb_->clear();
  (*tb_)[0] = dqad_.translation().q.segment(1, 3);
  q_ = q;
  qD_.setZero();
  lg_ = (*tb_)[0];
  next_ig_ = lg_;
  cg_ = lg_;
  current_ng_ = lg_;
  last_ng_ = lg_;
  next_ng_ = 0;
  current_ig_ = lg_;
  v_act_ = 0;
  v_goal_ = 0;
}

void CoSTPController::calculateControlPreliminaries(const Vector14d& q) {
  dqad_ = dq_dual_panda_->absolute_pose(q);
  // Current Plücker line in k_
  r_ = dq_dual_panda_->absolute_pose(q).P();
  ee_orientation_1_ = dqad_.P();
  l_ = ee_orientation_1_.conj() * k_ * ee_orientation_1_;
  AbsPJ_ = dq_dual_panda_->absolute_pose_jacobian(q);
  geomJac_ = geomJ(AbsPJ_, dqad_);
  relp_ = dq_dual_panda_->relative_pose(q);
  RelPJ_ = dq_dual_panda_->relative_pose_jacobian(q);
  // Calculating the angle_ between the plücker lines
  VectorXd t1 = l_.vec4();
  VectorXd t2 = lz_.vec4();
  angle_ = (180 / pi) * acos(t1.dot(t2));
  angle_ = abs(angle_);
}

void CoSTPController::relativePoseControl(MatrixXd& Jac, Vector14d& err) {
  // Calculate error
  rel_error_ = dqrd_.q - relp_.vec8();
  // Get Jacobian
  Jac = RelPJ_;
  // PseudoInverse
  MatrixXd robustInv = Jac.transpose() * pinv(Jac * Jac.transpose() +
                                              0.001 * MatrixXd::Identity(8, 8));
  err = robustInv * rel_error_;
}

void CoSTPController::getCurrentNominalGoal() {
  last_ng_ = current_ng_;
  current_ng_ = lg_ + next_ng_ * (cg_-lg_);
  next_ng_ += v_goal_*0.001/(cg_-lg_).norm();
}

Vector3d CoSTPController::getInstantaneousGoal() {
  Vector3d current_ig = next_ig_;
  if ((current_ng_ - current_ig).norm() < v_act_*0.001) {
    getCurrentNominalGoal();
  }
  next_ig_ = current_ig + v_act_*0.001*
      (current_ng_ - current_ig).normalized();
  current_ig_ = 0.9*current_ig_ + 0.1*current_ig;
  return current_ig_;
}

void CoSTPController::positionControlImpl(MatrixXd& Jac, Vector14d& err,
    const Vector3d& ig, const Vector3d& x_ff, const ControllerType::Type& ct) {
  abs_error_ = ig - dqad_.translation().q.segment(1, 3);
  Jac = geomJac_.middleRows(3, 3);
  // Funnel tracking
  double task_funnel_err_norm = pow(abs_error_.norm(),2);
  // MatrixXd robustInv = Jac.transpose() * pinv(Jac * Jac.transpose() +
  //                                             0.01 * MatrixXd::Identity(3, 3));
  MatrixXd funnel_Jac = 2 * abs_error_.transpose() * Jac;
  MatrixXd robustInv = funnel_Jac.transpose() * pinv(funnel_Jac * funnel_Jac.transpose() +
                                                0.01 * MatrixXd::Identity(1,1));
  err = robustInv * task_funnel_err_norm;
  // err = robustInv * abs_error_;
  double v = (Jac*err).norm();
  if (v > 0) {
    err *= abs_error_.norm()*1000 / v;
  }
  Vector3d x_c = gains_[ct]*(Jac*err)*0.001;
  Vector3d x_c_p = gains_[ct] * 0.001 * (Jac * ((MatrixXd::Identity(14, 14) - pinv(RelPJ_) * RelPJ_) * err));
  MatrixXd FullJac(11,14);
  FullJac << RelPJ_, geomJac_.middleRows(3, 3);
  MatrixXd fullRobustInv = FullJac.transpose() *
      pinv(FullJac * FullJac.transpose() + 0.01 * MatrixXd::Identity(11,11));
  Eigen::Matrix<double, 11, 1> x_ff_extended;
  x_ff_extended.setZero();
  x_ff_extended.tail(3) = x_ff;
  Vector14d dq_v_ff = fullRobustInv * x_ff_extended;
  double v_cmd = (Jac*dq_v_ff).norm();
  if (v_cmd > 0) {
    err += dq_v_ff*v_act_/(v_cmd*gains_[ct]);
  }
  Vector3d x_ff_ = 0.001*(Jac*dq_v_ff)*v_act_/v_cmd;
  Vector3d x_ff_p = 0.001 * (Jac *((MatrixXd::Identity(14, 14) - pinv(RelPJ_) *
                                    RelPJ_) * dq_v_ff)) * v_act_/v_cmd;
}

void CoSTPController::absolutePositionControl(MatrixXd& Jac, Vector14d& err,
    const Vector3d& cg, const double& v_max) {
  if (v_goal_ > v_act_) {
    v_act_ += 0.001*0.05;
    if (v_act_ > v_goal_) {
      v_act_ = v_goal_;
    }
  } else {
    v_act_ = v_goal_;
  }
  Vector3d inst_goal = getInstantaneousGoal();
  positionControlImpl(Jac, err, inst_goal, next_ig_-inst_goal,
                      ControllerType::ABSOLUTE_POSITION);
}

void CoSTPController::EETiltControl(MatrixXd& Jac, Vector14d& err) {
  // Calculate error
  VectorXd err_t = (l_ - lz_).vec4();
  e_n_ = pow(err_t.norm(), 2);
  // Get Jacobian
  MatrixXd Jr_r = AbsPJ_.topRows(4);
  DQ temp1 = k_ * r_;
  DQ temp2 = r_.conj() * k_;
  MatrixXd J_rz = temp1.haminus4() * C4() * Jr_r + temp2.hamiplus4() * Jr_r;
  Jac = -2 * err_t.transpose() * J_rz;
  // PseudoInverse
  MatrixXd robustInv = Jac.transpose() * pinv(Jac * Jac.transpose() +
                                              0.1 * MatrixXd::Identity(1, 1));
  err = robustInv * e_n_;
}

void CoSTPController::rotationalAdmittanceControl(MatrixXd& Jac, Vector14d& err,
                                                  const Vector14d& tau_ext) {
  Jac = geomJac_.topRows(3);
  err = -tau_ext;
}

void CoSTPController::wholeBodyCollisionAvoidance(MatrixXd& Jac,
    Vector14d& err, const Vector14d& q) {
  //mwba_.wholeBodyCollisionAvoidance(Jac, err, q);
  err.setZero();
  Jac = Eigen::Matrix<double, 6, 14>::Zero();
}

void CoSTPController::jointLimitAvoidanceControl(MatrixXd& Jac, Vector14d& err,
                                                 const Vector14d& q) {
  // // Get Jacobian
  Jac = (q - q_c_).transpose();
  /*auto range{q_max_ - q_min_};
  double threshold{0.4};
  for (size_t i = 0; i < q.size(); i++) {
    if (std::abs(Jac(0, i) / range(i)) < threshold) {
      Jac(0, i) = 0;
    }  // Add this if it makes jumps close to joint limits: else
       // {Jac(0,i) = (Jac(0,i)-threshold)/(0.5 - Jac(0,i)) }
  }*/
  // Calculate error
  VectorXd J_ssq = Jac.transpose().array().square();
  double s = 0.5 * J_ssq.sum();
  double lambda = 0.5;
  dot_s_ = -lambda * s;
  // PseudoInverse
  MatrixXd robustInv = Jac.transpose() * pinv(Jac * Jac.transpose() +
                                              0.001 * MatrixXd::Identity(1, 1));
  err = robustInv * dot_s_;

  // Task with single joint activation
  // double s {0.0};
  // std::vector<int> joints;
  // bool bv = boolJoint(q, joints);
  // if (bv == false) {
  //   Jac = MatrixXd::Zero(1,14);
  //   err = MatrixXd::Zero(1,14);
  // }
  // else {
  //   Vector14d q_tmp = Vector14d::Zero();
  //   Vector14d q_tmp_cc = Vector14d::Zero();
  //   q_tmp(joints.at(0)) = q(joints.at(0));
  //   q_tmp_cc(joints.at(0)) = q_c_(joints.at(0));
  //   Jac = (q_tmp - q_tmp_cc).transpose();
  //   VectorXd J_ssq = Jac.transpose().array().square();
  //   if (bv == true) {
  //     double s = 0.5 * J_ssq.sum();
  //   }
  //   else {
  //     double s = 0.0;
  //   }
  //   double lambda = 0.5;
  //   dot_s_ = -lambda * s;
  //   // PseudoInverse
  //   MatrixXd robustInv = Jac.transpose() * pinv(Jac * Jac.transpose() +
  //                                               0.001 * MatrixXd::Identity(1,
  //                                               1));
  //   err = robustInv * dot_s_;
  // }
}

void CoSTPController::fillBuffer(const Vector3d& goal) {
  std::unique_lock<std::recursive_mutex> lock(traj_buf_mutex_);
  if (!tb_->put(goal)) {
    ROS_ERROR("Couldn't put trajectory point into buffer.");
  }
  if (tb_->full()) {
    ready_for_next_point_ = false;
  }
}

Vector14d CoSTPController::followTrajectory(const Vector14d& tau_ext,
    const double& v_max) {
  static double min_motion = 2e-6;
  if (next_ng_ >= 1 || (v_act_ == 0 && next_ng_ == 0)) {
    bool got_point = false;
    {
      lg_ = cg_;
      std::lock_guard<std::recursive_mutex> lock(traj_buf_mutex_);
      if (!tb_->empty()) {
        cg_ = tb_->get();
        got_point = true;
      } else {
        next_ng_ = 0;
        v_act_ = 0;
      }
      ready_for_next_point_ = true;
    }
    if (got_point) {
      if (std::isnan(cg_[0]) || std::isnan(cg_[1]) || std::isnan(cg_[2])) {
        ROS_ERROR("Planner sent NaN.");
      }
      if ((cg_-lg_).norm() < 1e-6) {
        ROS_WARN_THROTTLE(1,
		    "Points sent by the planner are too close together.");
        cg_[2] += min_motion;
        min_motion = -min_motion;
      }
      const double v = v_max * (1 - catchup_reserve_);
      v_goal_ = std::min((cg_ - lg_).norm()*100, v);
      double acos_gamma, b;
      acos_gamma = (cg_-lg_).normalized().dot(current_ng_ - lg_);
      double radicand = std::pow(acos_gamma,2) + std::pow(v_goal_*0.001,2) -
                        std::pow((lg_-current_ng_).norm(),2);
      if (radicand >= 0) {
        b = acos_gamma + std::sqrt(radicand);
      } else {
        b = 0;
        ROS_WARN("Inconsistent trajectory detected.");
      }
      next_ng_ = b/(cg_-lg_).norm();
    }
  }
  qD_ = control(q_, qD_, cg_, tau_ext, v_max);
  q_ += qD_*0.001;
  return q_;
}

Vector14d CoSTPController::control(const Vector14d& q, const Vector14d& qD,
    Vector3d current_goal, const Vector14d& tau_ext, const double& vel) {
  MatrixXd Jac;
  MatrixXd Jac_total(0, 0);
  Vector14d err;
  dq_.setZero();
  calculateControlPreliminaries(q);
  if (switching_) {
    switchingOrder(q, qD);
  }
  for (int i = 0; i < controller_type_.size(); ++i) {
    Jac.setZero();
    switch (ControllerType(controller_type_[i])) {
      case ControllerType::RELATIVE_POSE: {
        relativePoseControl(Jac, err);
        break;
      }
      case ControllerType::WHOLE_BODY_AVOIDANCE: {
        wholeBodyCollisionAvoidance(Jac, err, q);
        break;
      }
      case ControllerType::ABSOLUTE_POSITION: {
        absolutePositionControl(Jac, err, current_goal, vel);
        break;
      }
      case ControllerType::EE_TILT: {
        EETiltControl(Jac, err);
        break;
      }
      case ControllerType::ROTATIONAL_ADMITTANCE: {
        rotationalAdmittanceControl(Jac, err, tau_ext);
        break;
      }
      case ControllerType::JOINT_LIMIT_AVOIDANCE: {
        jointLimitAvoidanceControl(Jac, err, q);
        break;
      }
    }
    if (i == 0) {
      dq_ += gains_[controller_type_[i]] * err;
      Jac_total = Jac;
    } else {
      dq_ += gains_[controller_type_[i]] *
             ((MatrixXd::Identity(14, 14) - pinv(Jac_total) * Jac_total) * err);
      MatrixXd tmp(Jac_total.rows() + Jac.rows(), Jac.cols());
      tmp << Jac_total, Jac;
      Jac_total = tmp;
    }
  }
  return dq_;
}

bool CoSTPController::tiltFunc(const double& absangle, const double& tilt_min,
                               const double& tilt_max) const {
  if (absangle > tilt_min && absangle < tilt_max)
    return true;
  else
    return false;
}

bool CoSTPController::boolFunc(const Vector14d& q, const Vector14d& qD) const {
  bool tmp { false };
  for (size_t i = 0; i < q.size(); i++) {
    tmp = (q(i) > q_min_(i) && q(i) < q_max_(i)) ||
          (q(i) <= q_min_(i) && qD(i) >= 0) ||
          (q(i) >= q_max_(i) && qD(i) <= 0);
    if (tmp) {
      return true;
    }
  }
  return false;
}

bool CoSTPController::boolJoint(const Vector14d& q, std::vector<int>& joints) const {
  bool tmp { false };
  for (size_t i = 0; i < q.size(); ++i) {
    if (q(i) <= q_min_(i) || q(i) >= q_max_(i)) {
      tmp = true;
      joints.push_back(i+1);
    }
  }
  return tmp;
}

void CoSTPController::switchingOrder(const Vector14d& q, const Vector14d& qD) {
  std::vector<ControllerType> tmp;
  tmp.push_back(ControllerType::RELATIVE_POSE);
  VectorXd t1 = l_.vec4();
  VectorXd t2 = lz_.vec4();
  double angle = (180 / pi) * acos(t1.dot(t2));
  angle = abs(angle);
  double tilt_min = 0;
  double tilt_max = 5;
  bool bvtilt = tiltFunc(angle, tilt_min, tilt_max);
  bool bv = boolFunc(q, qD);
  if (bvtilt == true && bv == true) {
    tmp.push_back(ControllerType::ABSOLUTE_POSITION);
    tmp.push_back(ControllerType::JOINT_LIMIT_AVOIDANCE);
    tmp.push_back(ControllerType::WHOLE_BODY_AVOIDANCE);
    tmp.push_back(ControllerType::EE_TILT);
  } else if (bvtilt == true && bv == false) {
    tmp.push_back(ControllerType::JOINT_LIMIT_AVOIDANCE);
    tmp.push_back(ControllerType::ABSOLUTE_POSITION);
    tmp.push_back(ControllerType::WHOLE_BODY_AVOIDANCE);
    tmp.push_back(ControllerType::EE_TILT);
  } else if (bvtilt == false && bv == true) {
    tmp.push_back(ControllerType::EE_TILT);
    tmp.push_back(ControllerType::WHOLE_BODY_AVOIDANCE);
    tmp.push_back(ControllerType::ABSOLUTE_POSITION);
    tmp.push_back(ControllerType::JOINT_LIMIT_AVOIDANCE);
  } else if (bvtilt == false && bv == false) {
    tmp.push_back(ControllerType::EE_TILT);
    tmp.push_back(ControllerType::WHOLE_BODY_AVOIDANCE);
    tmp.push_back(ControllerType::JOINT_LIMIT_AVOIDANCE);
    tmp.push_back(ControllerType::ABSOLUTE_POSITION);
  }
  controller_type_ = tmp;
}

MatrixXd geomJ(const MatrixXd& absoluteposeJ, const DQ& absolutepose) {
  Matrix<double, 8, 1> v;
  Matrix<double, 3, 4> CJ4_2_J3;
  MatrixXd J;
  v << -1, 1, 1, 1, -1, 1, 1, 1;
  MatrixXd C8 = v.array().matrix().asDiagonal();
  MatrixXd C4m = -C8.block(0, 0, 4, 4);
  CJ4_2_J3 << 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  DQ xm = absolutepose;
  if (absoluteposeJ.cols() == 14) {
    J.resize(6, 14);
    J.block(0, 0, 3, 14) =
        CJ4_2_J3 * 2 * xm.P().conj().haminus4() * absoluteposeJ.topRows(4);
    J.block(3, 0, 3, 14) =
        CJ4_2_J3 * 2 *
        (xm.D().hamiplus4() * C4m * absoluteposeJ.topRows(4) +
         xm.P().conj().haminus4() * absoluteposeJ.middleRows(4, 4));
  } else {
    J.resize(6, 7);
    J.block(0, 0, 3, 7) =
        CJ4_2_J3 * 2 * xm.P().conj().haminus4() * absoluteposeJ.topRows(4);
    J.block(3, 0, 3, 7) =
        CJ4_2_J3 * 2 *
        (xm.D().hamiplus4() * C4m * absoluteposeJ.topRows(4) +
         xm.P().conj().haminus4() * absoluteposeJ.middleRows(4, 4));
  }
  return J;
}

}  // namespace bimanual_planning_ros
