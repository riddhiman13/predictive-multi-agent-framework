#pragma once

#include <mutex>
#include <thread>

#include <Eigen/Dense>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <bimanual_planning_ros/obstacle.h>

typedef Eigen::Matrix<double, 14, 1> Vector14d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
using Eigen::MatrixXd;

using ghostplanner::cfplanner::Obstacle;

namespace bimanual_planning_ros {

class MoveitWholeBodyAvoidance {
 public:
  ~MoveitWholeBodyAvoidance();
  void addObstaclesLeft(const std::vector<moveit_msgs::CollisionObject>& msg);
  void addObstaclesRight(const std::vector<moveit_msgs::CollisionObject>& msg);
  void init(const Eigen::Isometry3d& base_tf_l,
            const Eigen::Isometry3d& base_tf_r);
  void moveObstaclesLeft(const std::vector<moveit_msgs::CollisionObject>& msg);
  void moveObstaclesRight(const std::vector<moveit_msgs::CollisionObject>& msg);
  void setWholeBodyDistanceThresh(double distance_0);
  void wholeBodyCollisionAvoidance(MatrixXd& Jac, Vector14d& err,
                                   const Vector14d& q);
 private:
  size_t next_marker_id_;
  double distance_0_;
  double wb_avoidance_max_distance_ = 0.3;
  Vector14d err_, q_;
  Eigen::MatrixXd Jac_;
  Eigen::Isometry3d base_tf_l_, base_tf_r_;
  std::recursive_mutex data_mutex_, q_mutex_, obs_mutex_l_, obs_mutex_r_;
  std::thread calculation_thread_;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  std::vector<moveit_msgs::CollisionObject> obstacles_l_, obstacles_r_;
  bool has_obstacles_ = false;
  bool update_obstacles_l_ = false, update_obstacles_r_ = false;
  static double avoidanceResponse(const double& distance,
                                  const double& distance_0);
  void calculationThreadFcn();
  void visualiseForce(const Eigen::Vector3d& start_point,
      const Eigen::Vector3d& force, const Eigen::Isometry3d tf);
  Vector7d wholeBodyCollisionAvoidanceTorques(
    const planning_scene::PlanningScene& scene,
    const std::string &planning_group, bool left,
    Eigen::Ref<Eigen::Matrix<double, 3, 7>> Jac);
};

}
