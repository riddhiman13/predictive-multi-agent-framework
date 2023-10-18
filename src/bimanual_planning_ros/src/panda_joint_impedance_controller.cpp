// Copyright (c) 2020 Lehrstuhl für Robotik und Sysstemintelligenz, TU München
#include <bimanual_planning_ros/panda_joint_impedance_controller.h>

#include <cmath>

#include <franka/robot_state.h>

#include <bimanual_planning_ros/damping_design.h>

typedef Eigen::Matrix<double, 7, 7> Matrix7d;

namespace bimanual_planning_ros {

bool PandaJointImpedanceController::init(hardware_interface::RobotHW* robot_hw,
    const std::string& arm_id, const std::vector<std::string>& joint_names) {
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("DualArmSwitchingController: Error getting model interface"
                     " from hardware");
    return false;
  }
  try {
    model_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmSwitchingController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("DualArmSwitchingController: Error getting state interface"
                     " from hardware");
    return false;
  }
  try {
    state_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmSwitchingController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }
  auto* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmSwitchingController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joints_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DualArmSwitchingController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }
  return true;
}

Matrix7d jointDampingDesign(
    const Matrix7d& stiffness,
    const Matrix7d& damping_ratio,
    const Matrix7d& inertia) {
  // Compute Cartesian Mass inverse, previously adding motor intertia B
  Vector7d b, k_T;  // NOLINT (readability-identifier-naming)
  // Motor mass matrix
  Matrix7d inertia_hat;
  b << 0.6057, 0.6057, 0.4625, 0.4625, 0.2055, 0.2055, 0.2055;
  k_T << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 2.5;
  Matrix7d B, K_T, I_K_T_inv;  // NOLINT (readability-identifier-naming)
  B = b.asDiagonal();
  K_T = k_T.asDiagonal();
  I_K_T_inv = Eigen::MatrixXd::Identity(7, 7) + K_T;
  I_K_T_inv = I_K_T_inv.inverse();
  inertia_hat << inertia + I_K_T_inv * B;
  return panda_controllers_impl::factorizationDesign<7>(
      stiffness, damping_ratio, inertia_hat);
}

void PandaJointImpedanceController::control(const Vector7d& q_d,
                                            const Vector7d& qD_d) {
  franka::RobotState robot_state = state_->getRobotState();
  std::array<double, 7> coriolis_array = model_->getCoriolis();
  std::array<double, 49> inertia_array = model_->getMass();
  Eigen::Map<Vector7d> coriolis(coriolis_array.data());
  Eigen::Map<Matrix7d> inertia(inertia_array.data());
  Eigen::Map<Vector7d> q(robot_state.q.data());
  Eigen::Map<Vector7d> qD(robot_state.dq.data());
  static Matrix7d damping_ratio = Matrix7d::Identity()*0.8;
  static Vector7d stiffness;
  stiffness << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
  Matrix7d designed_damping = jointDampingDesign(
      stiffness_scale_ * stiffness.asDiagonal(), damping_ratio, inertia);
  auto tau_d =
        coriolis +
        stiffness_scale_ * stiffness.asDiagonal() * (q_d - q) +
        designed_damping * (qD_d - qD);
  for (size_t i = 0; i < 7; ++i) {
    joints_[i].setCommand(tau_d[i]);
  }
}

void PandaJointImpedanceController::setStiffnessScale(
    const double& stiffness_scale) {
  stiffness_scale_ = stiffness_scale;
}

double PandaJointImpedanceController::limitQD(Vector7d qD_d,
    const double& v_cart) {
  double scale = 1;
  Eigen::Matrix<double, 6, 7> J(model_->getZeroJacobian(
      franka::Frame::kEndEffector).data());
  Vector7d q(state_->getRobotState().q.data());
  double v = (J*qD_d).head(3).norm();
  if (v_cart > 0) {
    if (v > v_cart) {
      scale = v_cart/v;
      qD_d *= scale;
    }
  }
  Vector7d qD_max;
  qD_max << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61;
  qD_max *= 0.99;
  double max_rate = 1;
  for (int i=0;i<7;++i) {
    if (std::abs(qD_d[i]) > qD_max[i]) {
      max_rate = std::max(max_rate, std::abs(qD_d[i])/qD_max[i]);
    }
  }
  return scale/max_rate;
}

}  // namespace bimanual_planning_ros
