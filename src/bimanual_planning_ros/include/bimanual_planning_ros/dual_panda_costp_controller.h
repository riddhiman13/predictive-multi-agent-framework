// Copyright (c) 2020 Lehrstuhl für Robotik und Sysstemintelligenz, TU München
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <bimanual_planning_ros/CollisionObjects.h>
#include <bimanual_planning_ros/ControllerInfo.h>
#include <bimanual_planning_ros/ControllerParams.h>
#include <bimanual_planning_ros/controller_type.h>
#include <bimanual_planning_ros/costp_controller.h>
#include <bimanual_planning_ros/JointMotion.h>
#include <bimanual_planning_ros/panda_joint_impedance_controller.h>
#include <bimanual_planning_ros/Position.h>
#include <bimanual_planning_ros/safe_and_fast_publisher.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 14, 1> Vector14d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

namespace bimanual_planning_ros {

enum class ControlState {JOINT_MOTION, IDLE, SWITCHING_CONTROL,
                         INIT_SWITCHING_CONTROL};

class DualPandaCoSTPController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  DualPandaCoSTPController();
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
 private:
  CoSTPController controller_;
  std::string planning_frame_;
  Matrix3d B_R_O_l_, B_R_O_r_, O_R_B_l_, O_R_B_r_;
  Vector3d r_B_O_l_, r_B_O_r_;
  std::mutex control_mutex_;
  Vector14d dq_, q_d_, qD_d_;
  Vector6d contact_wrench_;
  Vector3d current_goal_;
  std::vector<Obstacle> obstacles_;
  double velocity_;
  PandaJointImpedanceController left_controller_, right_controller_;
  ControlState state_;
  SafeAndFastPublisher p_pub_, info_pub_, contact_wrench_pub_;
  ros::ServiceServer joint_motion_server_;
  franka_hw::TriggerRate publish_rate_;
  bool published_position_ = false, new_point_ = false;
  bool initArm(hardware_interface::RobotHW* robot_hw,
               const std::string& arm_id,
               const std::vector<std::string>& joint_names,
               FrankaDataContainer& arm_data);
  void controlArm(FrankaDataContainer& d);
  ros::Subscriber sub_target_pose_, sub_params_, obstacle_add_sub_l_,
      obstacle_add_sub_r_, obstacle_move_sub_l_, obstacle_move_sub_r_;
  void addObstaclesLeftCallback(const CollisionObjects& msg);
  void addObstaclesRightCallback(const CollisionObjects& msg);
  void moveObstaclesLeftCallback(const CollisionObjects& msg);
  void moveObstaclesRightCallback(const CollisionObjects& msg);
  void targetPoseCallback(const Position& msg);
  void controllerParamsCallback(const ControllerParams& p);
  bool jointMotionCallback(JointMotion::Request& req,
                           JointMotion::Response& res);
  void publishInfoError();
  void publishPosition();
  void publishContactWrench();
  void setLowLevelControllerGoal();
  void initSwitchingControl();
  void printErrors();
  void computeContactWrench();
  Vector3d B_T_O_l(Vector3d p_0);
  Vector3d B_T_O_r(Vector3d p_0);
  Vector3d O_T_B_l(Vector3d p_B);
  Vector3d O_T_B_r(Vector3d p_B);
  Quaterniond B_T_O_l(Quaterniond q_0);
  Quaterniond B_T_O_r(Quaterniond q_0);
  Quaterniond O_T_B_l(Quaterniond q_B);
  Quaterniond O_T_B_r(Quaterniond q_B);
};

}  // namespace bimanual_planning_ros
