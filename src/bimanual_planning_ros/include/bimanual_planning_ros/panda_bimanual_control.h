#pragma once

#include <bimanual_planning_ros/ContactWrench.h>
#include <bimanual_planning_ros/Obstacles.h>
#include <bimanual_planning_ros/Position.h>
#include <bimanual_planning_ros/cf_manager.h>
#include <bimanual_planning_ros/dual_panda_interface.h>
#include <bimanual_planning_ros/end_condition.h>
#include <bimanual_planning_ros/obstacle.h>
#include <bimanual_planning_ros/parameter_manager.h>
#include <dynamic_reconfigure/client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

using Eigen::Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 14, 1> Vector14d;

using ghostplanner::cfplanner::Obstacle;

namespace bimanual_planning_ros {
class PandaBimanualPlanning {
 private:
  ros::NodeHandle rosnode_;
  ros::Publisher event_pub_, goal_pub_, dist_pub_, controller_params_pub_,
      obs_pub_, commanded_path_pub_, predicted_paths_pub_, error_recovery_pub_;
  ros::Subscriber contact_wrench_sub_, position_sub_, obstacle_sub_;
  ros::Timer task_timer_;
  bool planning_active_, open_loop_, init_params_, goal_finished_, switching_,
      displayed_goal_, visualize_commanded_path_, visualize_predicted_paths_,
      got_initial_pos_;
  int current_goal_;
  double time_step_, velocity_, frequency_ros_;
  Vector3d force_offset_, last_goal_, obst_pos_, obst_vel_, last_pos_;
  Vector14d q_goal_;
  Vector6d contact_wrench_, th_min_, th_max_;
  double gesture_th_, gripper_width_, force_feedback_, force_dead_zone_,
      low_level_gain_;
  std::string type_, goal_type_, robot_type_, message_, event_, end_condition_;
  ghostplanner::cfplanner::CfManager cf_manager_;
  Eigen::MatrixXd predicted_paths_x_;
  Eigen::MatrixXd predicted_paths_y_;
  Eigen::MatrixXd predicted_paths_z_;
  bool first_planning_run_ = true;
  ParameterManager check_pm_, init_pm_, goal_pm_;
  std::vector<Obstacle> obstacles_;
  double k_attr_, k_circ_, k_repel_, k_damp_, k_manip_, k_repel_body_,
      k_goal_dist_, k_path_len_, k_safe_dist_, k_workspace_, gain_, radius_,
      approach_dist_, detect_shell_rad_;
  Vector6d des_ws_limits_;
  Vector7d q_goal_left_, q_goal_right_;
  size_t num_agents_ee_, num_agents_body_, max_prediction_steps_,
      prediction_freq_multiple_;
  EndCondition ec_;
  XmlRpc::XmlRpcValue goals_;
  std::vector<std::string> controller_type_;
  std::vector<double> gains_, reflex_tau_max_, reflex_F_max_;
  std::unique_ptr<RobotInterface> ri_;
  void contactWrenchCallback(const ContactWrench &msg);
  void obstacleCallback(const Obstacles &msg);
  void initROSConfig();
  void planCallback(const Position &p);
  void errorRecovery();
  void getParameters();
  void getGoalParameters(const XmlRpc::XmlRpcValue &v);
  void initRobotArmConfig();
  void startLogging();
  void stopLogging();
  void taskCallback(const ros::TimerEvent& event);
  void visualize_commanded_path(
      const std::vector<Eigen::Vector3d> &poses) const;
  void visualize_predicted_path(const std::vector<Eigen::Vector3d> &poses,
                                int agent_num, int best_agent_num) const;
  std::chrono::steady_clock::time_point end_callback;

 public:
  PandaBimanualPlanning();
  ~PandaBimanualPlanning();
  bool hasContact();
  void waitForGesture(double F_th = 8);
  void setThresholds(const Eigen::Matrix<double, 6, 1> &F_min,
                     const Eigen::Matrix<double, 6, 1> &F_max);
  void openGripper(bool left, double width);
  void openGrippers(double width);
  void closeGripper(bool left);
  void closeGrippers();
  void jointMotion(const Vector14d &goal, const double &v = 0.5);
  void start();
  Eigen::Vector3d getContactPointEstimation(Eigen::Vector3d n,
                                            Eigen::Vector3d p);
};
}  // namespace bimanual_planning_ros
