/*
   Class for circular field agents
*/
#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "bimanual_planning_ros/helper_functions.h"
#include "bimanual_planning_ros/obstacle.h"
#include "eigen3/Eigen/Dense"

namespace ghostplanner {
namespace cfplanner {

template <typename T>
class CopyableAtomic {
 public:
  operator T() const {return data_.load();};
  T operator=(const T& other) {data_.store(other); return data_.load();};
  CopyableAtomic& operator=(const CopyableAtomic& other) {
      data_.store(other.data_.load()); return *this;};
  CopyableAtomic(const CopyableAtomic& other) {
      data_.store(other.data_.load());};
  CopyableAtomic(const T& data) {data_.store(data);};
  CopyableAtomic() {};
 private:
  std::atomic<T> data_;
};

class CfAgent {
 protected:
  int id_;
  std::vector<Eigen::Vector3d> pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
  Eigen::Vector3d init_pos_;
  Eigen::Vector3d g_pos_;
  Eigen::Vector3d force_;
  double detect_shell_rad_;
  double mass_;
  double rad_;
  double vel_max_;
  double min_obs_dist_;
  double approach_dist_;
  CopyableAtomic<bool> run_prediction_;
  CopyableAtomic<bool> running_;
  CopyableAtomic<bool> finished_;
  std::vector<Obstacle> obstacles_;
  std::vector<bool> known_obstacles_;
  std::vector<Eigen::Vector3d> field_rotation_vecs_;
  double prediction_time_;
  bool reached_goal_;

 public:
  enum Type {
    REAL_AGENT,
    GOAL_HEURISTIC,
    OBSTACLE_HEURISTIC,
    GOAL_OBSTACLE_HEURISTIC,
    VEL_HEURISTIC,
    RANDOM_AGENT,
    HAD_HEURISTIC,
    UNDEFINED
  };
  CfAgent(const int id, const Eigen::Vector3d agent_pos,
          const Eigen::Vector3d goal_pos, const double detect_shell_rad,
          const double agent_mass, const double radius,
          const double velocity_max, const double approach_dist,
          const int num_obstacles, const std::vector<Obstacle> obstacles)
      : id_{id},
        pos_{agent_pos},
        vel_{0.01, 0.0, 0.0},
        acc_{0.0, 0.0, 0.0},
        init_pos_{0.0, 0.0, 0.0},
        g_pos_{goal_pos},
        detect_shell_rad_{detect_shell_rad},
        min_obs_dist_{detect_shell_rad},
        force_{0.0, 0.0, 0.0},
        mass_{agent_mass},
        rad_{radius},
        vel_max_{velocity_max},
        approach_dist_{approach_dist},
        run_prediction_{false},
        running_{false},
        finished_{false},
        reached_goal_{false},
        obstacles_{obstacles} {
    Eigen::Vector3d default_rot_vec{0.0, 0.0, 1.0};
    for (size_t i = 0; i < num_obstacles; i++) {
      field_rotation_vecs_.push_back(default_rot_vec);
      known_obstacles_.push_back(false);
    }
  };
  CfAgent() = default;
  virtual ~CfAgent() = default;
  CfAgent(const CfAgent &) = default;
  CfAgent(CfAgent &&) noexcept = default;
  CfAgent &operator=(const CfAgent &) = default;
  CfAgent &operator=(CfAgent &&) noexcept = default;
  Eigen::Vector3d getFirstPosition() const;
  Eigen::Vector3d getPosition(int id) const;
  Eigen::Vector3d getLatestPosition() const;
  Eigen::Vector3d getInitialPosition() const { return init_pos_; };
  std::vector<Eigen::Vector3d> getPath() const { return pos_; };
  double getPathLength() const;
  Eigen::Vector3d getGoalPosition() const { return g_pos_; };
  virtual void setPosition(Eigen::Vector3d position);
  void setInitalPosition(Eigen::Vector3d position);
  void resetForce() { force_ << 0.0, 0.0, 0.0; };
  Eigen::Vector3d getForce() { return force_; };
  Eigen::Vector3d getVelocity() { return vel_; };
  double getDistFromGoal() const {
    return (g_pos_ - this->getLatestPosition()).norm();
  };
  int getNumPredictionSteps() const { return pos_.size(); };
  int getAgentID() const { return id_; };
  bool getRunningStatus() { return running_; };
  double getMinObsDist() { return min_obs_dist_; };
  double getPredictionTime() { return prediction_time_; };
  bool getReachedGoal() { return reached_goal_; };
  void startPrediction() { run_prediction_ = true; };
  void stopPrediction() { run_prediction_ = false; };
  void shutdownAgent() { finished_ = true; };
  void resetMinObsDist() { min_obs_dist_ = detect_shell_rad_; };
  void setVelocity(const Eigen::Vector3d &velocity);
  void setObstacles(const std::vector<Obstacle> &obstacles,
                    const std::vector<bool> real_known_obstacles);
  void setGoal(const Eigen::Vector3d &goal) { g_pos_ = goal; };
  virtual void circForce(const std::vector<Obstacle> &obstacles,
                         const double k_circ);
  void repelForce(const std::vector<Obstacle> &obstacles, const double k_repel);
  void attractorForce(const double k_attr, const double k_damp,
                      const double k_goal_scale);
  double attractorForceScaling(const std::vector<Obstacle> &obstacles);
  Eigen::Vector3d bodyForce(const std::vector<Obstacle> &obstacles,
                            const double k_repel);
  void updatePositionAndVelocity(const double delta_t);
  void predictObstacles(const double delta_t);
  virtual void cfPlanner(const std::vector<Eigen::Vector3d> &manip_map,
                         const std::vector<Obstacle> &obstacles,
                         const double k_attr, const double k_circ,
                         const double k_repel, const double k_damp,
                         const double k_manip, const double delta_t,
                         const int steps = 1);
  void cfPrediction(const std::vector<Eigen::Vector3d> &manip_map,
                    const double k_attr, const double k_circ,
                    const double k_repel, const double k_damp,
                    const double k_manip, const double delta_t,
                    const size_t max_prediction_steps);
  void reset();
  virtual Eigen::Vector3d currentVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
      const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
      const int obstacle_id,
      const std::vector<Eigen::Vector3d> field_rotation_vecs) const {};
  virtual Eigen::Vector3d calculateRotationVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
      const std::vector<Obstacle> &obstacles, const int obstacle_id) const {};

  double evalObstacleDistance(const std::vector<Obstacle> &obstacles) const;
  void manipulabilityForce(const std::vector<Eigen::Vector3d> &manip_map,
                           double k_manip, double offset, double resolution);
  virtual bool isRealAgent() { return false; };
  virtual std::unique_ptr<CfAgent> makeCopy() {
    return std::unique_ptr<CfAgent>(new CfAgent(*this));
  };
  virtual Type getAgentType() { return UNDEFINED; };
};

class RealCfAgent : public CfAgent {
 public:
  RealCfAgent(const int id, const Eigen::Vector3d agent_pos,
              const Eigen::Vector3d goal_pos, const double detect_shell_rad,
              const double agent_mass, const double radius,
              const double velocity_max, const double approach_dist,
              const int num_obstacles)
      : CfAgent(id, agent_pos, goal_pos, detect_shell_rad, agent_mass, radius,
                velocity_max, approach_dist, num_obstacles,
                std::vector<Obstacle>()){};
  RealCfAgent() = default;

  Eigen::Vector3d currentVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
      const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
      const int obstacle_id,
      const std::vector<Eigen::Vector3d> field_rotation_vecs,
      const CfAgent &agent) const;
  Eigen::Vector3d calculateRotationVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
      const std::vector<Obstacle> &obstacles, const int obstacle_id,
      const CfAgent &agent) const;
  void circForce(const std::vector<Obstacle> &obstacles, const double k_circ,
                 const CfAgent &agent);
  void cfPlanner(const std::vector<Eigen::Vector3d> &manip_map,
                 const std::vector<Obstacle> &obstacles, const CfAgent &agent,
                 const double k_attr, const double k_circ, const double k_repel,
                 const double k_damp, const double k_manip,
                 const double delta_t, const int steps = 1);
  void setPosition(Eigen::Vector3d position) override;
  bool isRealAgent() override { return true; };
  Type getAgentType() override { return REAL_AGENT; };
  std::unique_ptr<CfAgent> makeCopy() override {
    return std::unique_ptr<CfAgent>(new RealCfAgent(*this));
  };
  std::vector<bool> getKnownObstacles() { return known_obstacles_; };
};

// Agent avoids obstacles in the direction of the goal pose
class GoalHeuristicCfAgent : public CfAgent {
 public:
  GoalHeuristicCfAgent(const int id, const Eigen::Vector3d agent_pos,
                       const Eigen::Vector3d goal_pos,
                       const double detect_shell_rad, const double agent_mass,
                       const double radius, const double velocity_max,
                       const double approach_dist, const int num_obstacles,
                       const std::vector<Obstacle> obstacles)
      : CfAgent(id, agent_pos, goal_pos, detect_shell_rad, agent_mass, radius,
                velocity_max, approach_dist, num_obstacles, obstacles){};
  Eigen::Vector3d currentVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
      const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
      const int obstacle_id,
      const std::vector<Eigen::Vector3d> field_rotation_vecs) const override;
  Eigen::Vector3d calculateRotationVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
      const std::vector<Obstacle> &obstacles,
      const int obstacle_id) const override;
  Type getAgentType() override { return GOAL_HEURISTIC; };
  std::unique_ptr<CfAgent> makeCopy() override {
    return std::unique_ptr<CfAgent>(new GoalHeuristicCfAgent(*this));
  };
};

// Agent avoids active obstacle in the direction which is opposite to the
// direction from the active obstacle to its closest obstacle, i.e., steering
// away from the next but one obstacle.
class ObstacleHeuristicCfAgent : public CfAgent {
 public:
  ObstacleHeuristicCfAgent(const int id, const Eigen::Vector3d agent_pos,
                           const Eigen::Vector3d goal_pos,
                           const double detect_shell_rad,
                           const double agent_mass, const double radius,
                           const double velocity_max,
                           const double approach_dist, const int num_obstacles,
                           const std::vector<Obstacle> obstacles)
      : CfAgent(id, agent_pos, goal_pos, detect_shell_rad, agent_mass, radius,
                velocity_max, approach_dist, num_obstacles, obstacles){};
  Eigen::Vector3d currentVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
      const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
      const int obstacle_id,
      const std::vector<Eigen::Vector3d> field_rotation_vecs) const override;
  Eigen::Vector3d calculateRotationVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
      const std::vector<Obstacle> &obstacles,
      const int obstacle_id) const override;
  Type getAgentType() override { return OBSTACLE_HEURISTIC; };
  std::unique_ptr<CfAgent> makeCopy() override {
    return std::unique_ptr<CfAgent>(new ObstacleHeuristicCfAgent(*this));
  };
};

// Evading direction of agent is between the GoalHeuristicAgent and the
// ObstacleHeuristicAgent
class GoalObstacleHeuristicCfAgent : public CfAgent {
 public:
  GoalObstacleHeuristicCfAgent(const int id, const Eigen::Vector3d agent_pos,
                               const Eigen::Vector3d goal_pos,
                               const double detect_shell_rad,
                               const double agent_mass, const double radius,
                               const double velocity_max,
                               const double approach_dist,
                               const int num_obstacles,
                               const std::vector<Obstacle> obstacles)
      : CfAgent(id, agent_pos, goal_pos, detect_shell_rad, agent_mass, radius,
                velocity_max, approach_dist, num_obstacles, obstacles){};
  Eigen::Vector3d currentVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
      const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
      const int obstacle_id,
      const std::vector<Eigen::Vector3d> field_rotation_vecs) const override;
  Eigen::Vector3d calculateRotationVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
      const std::vector<Obstacle> &obstacles,
      const int obstacle_id) const override;
  Type getAgentType() override { return GOAL_OBSTACLE_HEURISTIC; };
  std::unique_ptr<CfAgent> makeCopy() override {
    return std::unique_ptr<CfAgent>(new GoalObstacleHeuristicCfAgent(*this));
  };
};

// Agent avoids obstacle in direction of its current velocity (see Ataka RAL
// paper 2018)
class VelHeuristicCfAgent : public CfAgent {
 public:
  VelHeuristicCfAgent(const int id, const Eigen::Vector3d agent_pos,
                      const Eigen::Vector3d goal_pos,
                      const double detect_shell_rad, const double agent_mass,
                      const double radius, const double velocity_max,
                      const double approach_dist, const int num_obstacles,
                      const std::vector<Obstacle> obstacles)
      : CfAgent(id, agent_pos, goal_pos, detect_shell_rad, agent_mass, radius,
                velocity_max, approach_dist, num_obstacles, obstacles){};
  Eigen::Vector3d currentVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
      const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
      const int obstacle_id,
      const std::vector<Eigen::Vector3d> field_rotation_vecs) const override;
  Eigen::Vector3d calculateRotationVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
      const std::vector<Obstacle> &obstacles,
      const int obstacle_id) const override;
  Type getAgentType() override { return VEL_HEURISTIC; };
  std::unique_ptr<CfAgent> makeCopy() override {
    return std::unique_ptr<CfAgent>(new VelHeuristicCfAgent(*this));
  };
};

// Agent avoids obstacle in a random direction
class RandomCfAgent : public CfAgent {
 protected:
  std::vector<Eigen::Vector3d> random_vecs_;

 public:
  RandomCfAgent(const int id, const Eigen::Vector3d agent_pos,
                const Eigen::Vector3d goal_pos, const double detect_shell_rad,
                const double agent_mass, const double radius,
                const double velocity_max, const double approach_dist,
                const int num_obstacles, const std::vector<Obstacle> obstacles)
      : CfAgent(id, agent_pos, goal_pos, detect_shell_rad, agent_mass, radius,
                velocity_max, approach_dist, num_obstacles, obstacles) {
    // For recreating the simulation with the same behavior
    // saveRandomVecToFile(num_obstacles);
    for (size_t i = 0; i < num_obstacles; i++) {
      Eigen::Vector3d random_vec = makeRandomVector();
      random_vec.normalize();
      random_vecs_.push_back(random_vec);
    }
    Vector3d random_vec;
  };
  void saveRandomVecToFile(const int num_obstacles);
  Eigen::Vector3d currentVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
      const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
      const int obstacle_id,
      const std::vector<Eigen::Vector3d> field_rotation_vecs) const override;
  Eigen::Vector3d calculateRotationVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
      const std::vector<Obstacle> &obstacles,
      const int obstacle_id) const override;
  Type getAgentType() override { return RANDOM_AGENT; };
  std::unique_ptr<CfAgent> makeCopy() override {
    return std::unique_ptr<CfAgent>(new RandomCfAgent(*this));
        };
};

class HadHeuristicCfAgent : public CfAgent {
 public:
  HadHeuristicCfAgent(const int id, const Eigen::Vector3d agent_pos,
                      const Eigen::Vector3d goal_pos,
                      const double detect_shell_rad, const double agent_mass,
                      const double radius, const double velocity_max,
                      const double approach_dist, const int num_obstacles,
                      const std::vector<Obstacle> obstacles)
      : CfAgent(id, agent_pos, goal_pos, detect_shell_rad, agent_mass, radius,
                velocity_max, approach_dist, num_obstacles, obstacles){};
  Eigen::Vector3d currentVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
      const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
      const int obstacle_id,
      const std::vector<Eigen::Vector3d> field_rotation_vecs) const override;
  Eigen::Vector3d calculateRotationVector(
      const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
      const std::vector<Obstacle> &obstacles,
      const int obstacle_id) const override;
  Type getAgentType() override { return HAD_HEURISTIC; };
  std::unique_ptr<CfAgent> makeCopy() override {
    return std::unique_ptr<CfAgent>(new HadHeuristicCfAgent(*this));
  };
};
}  // namespace cfplanner
}  // namespace ghostplanner
