/*
   Dual quaternion bimanual controller
*/
#pragma once

#include <bimanual_planning_ros/CollisionObjects.h>
#include <bimanual_planning_ros/ControllerParams.h>
#include <bimanual_planning_ros/JointMotion.h>
#include <bimanual_planning_ros/Position.h>
#include <bimanual_planning_ros/costp_controller.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <mutex>
#include <string>

#include "dqrobotics/DQ.h"
#include "dqrobotics/interfaces/vrep/DQ_VrepInterface.h"
#include "dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h"
#include "dqrobotics/robot_modeling/DQ_SerialManipulator.h"

namespace ghostplanner {
namespace dqcontroller {

enum class ControlState {
  JOINT_MOTION,
  IDLE,
  SWITCHING_CONTROL,
  INIT_SWITCHING_CONTROL
};

class VrepController {
 private:
  ros::NodeHandle rosnode_;
  ros::Subscriber planner_sub_, params_sub_, obstacle_add_sub_l_,
      obstacle_add_sub_r_, obstacle_move_sub_l_, obstacle_move_sub_r_;
  ros::Publisher pose_pub_;
  ros::Publisher start_obstacles_pub_;
  ros::Publisher abs_pose_pub_;
  ros::Publisher joint_l_pub_;
  ros::Publisher joint_r_pub_;
  ros::ServiceServer joint_motion_server_;
  DQ_VrepInterface vi_;
  DQ_CooperativeDualTaskSpace panda_bimanual_;
  ControlState state_;
  std::mutex control_mutex_;
  Vector3d planned_pos_;
  Vector14d q_coop_curr_;
  Vector14d q_min_;
  Vector14d q_max_;
  Vector14d q_joint_motion_;
  Vector7d q_l_curr_;
  Vector7d q_r_curr_;
  Vector14d dq_;
  const double delta_t_;
  double velocity_;
  bool published_position_ = false;
  const std::vector<std::string> joint_names_l_;
  const std::vector<std::string> joint_names_r_;
  const std::string robot_l_, robot_r_;
  bimanual_planning_ros::CoSTPController controller_;
  void publishPosition();
  void publishAbsPose();
  void publishJointStates();
  void starting();
  void initSwitchingControl();
  void controllerParamsCallback(
      const bimanual_planning_ros::ControllerParams &p);
  bool jointMotionCallback(bimanual_planning_ros::JointMotion::Request& req,
                           bimanual_planning_ros::JointMotion::Response& res);
  void addObstaclesLeftCallback(
      const bimanual_planning_ros::CollisionObjects& msg);
  void addObstaclesRightCallback(
      const bimanual_planning_ros::CollisionObjects& msg);
  void moveObstaclesLeftCallback(
      const bimanual_planning_ros::CollisionObjects& msg);
  void moveObstaclesRightCallback(
      const bimanual_planning_ros::CollisionObjects& msg);
  void targetPoseCallback(const bimanual_planning_ros::Position &msg);
  std::vector<Obstacle> obstacles_;

 public:
  VrepController(DQ_VrepInterface &vi,
      DQ_CooperativeDualTaskSpace &panda_bimanual,
      const std::vector<std::string> &joint_names_l,
      const std::vector<std::string> &joint_names_r, const std::string& robot_l,
      const std::string& robot_r, const double delta_t);
  ~VrepController() = default;
  void init(Vector3d r_B_O_r, Quaterniond R_B_O_r, Vector3d r_B_O_l,
            Quaterniond R_B_O_l);
  void update();
};

}  // namespace dqcontroller
}  // namespace ghostplanner
