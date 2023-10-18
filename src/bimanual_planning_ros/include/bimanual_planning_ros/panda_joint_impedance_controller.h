// Copyright (c) 2020 Lehrstuhl für Robotik und Sysstemintelligenz, TU München
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

typedef Eigen::Matrix<double, 7, 1> Vector7d;

namespace bimanual_planning_ros {

struct FrankaDataContainer {

};

class PandaJointImpedanceController {
 public:
  void control(const Vector7d& q_d, const Vector7d& qD_d);
  bool init(hardware_interface::RobotHW* robot_hw, const std::string& arm_id,
            const std::vector<std::string>& joint_names);
  double limitQD(Vector7d qD_d, const double& v_cart);
  void setStiffnessScale(const double& stiffness_scale);
  const double* getQ() {return state_->getRobotState().q.data();};
  const double* getQD() {return state_->getRobotState().dq.data();};
  const franka::RobotState& getRobotState() {return state_->getRobotState();};
  const franka_hw::FrankaModelHandle& getModel() {return *model_;};
 private:
  std::unique_ptr<franka_hw::FrankaStateHandle> state_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_;
  std::vector<hardware_interface::JointHandle> joints_;
  double stiffness_scale_{1};
};

}  // namespace bimanual_planning_ros
