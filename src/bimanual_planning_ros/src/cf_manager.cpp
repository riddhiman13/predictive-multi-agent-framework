#include "bimanual_planning_ros/cf_manager.h"

#include <omp.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>

using Eigen::Vector3d;
using std::vector;

namespace ghostplanner {
namespace cfplanner {
CfManager::CfManager(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
    const double delta_t, const std::vector<Obstacle> &obstacles,
    const std::vector<double> &k_a_ee, const std::vector<double> &k_c_ee,
    const std::vector<double> &k_r_ee, const std::vector<double> &k_d_ee,
    const std::vector<double> &k_manip, const std::vector<double> &k_r_force,
    const double velocity_max, const double approach_dist,
    const double detect_shell_rad, const size_t max_prediction_steps,
    const size_t prediction_freq_multiple, const double agent_mass,
    const double radius)
    : goal_pos_(goal_pos),
      k_a_ee_{k_a_ee},
      k_c_ee_{k_c_ee},
      k_r_ee_{k_r_ee},
      k_d_ee_{k_d_ee},
      k_manip_{k_manip},
      k_r_force_{k_r_force},
      run_prediction_{false},
      approach_dist_{approach_dist},
      real_ee_agent_(0, agent_pos, goal_pos, detect_shell_rad, agent_mass,
                     radius, velocity_max, approach_dist, obstacles.size()) {
  init(goal_pos, delta_t, obstacles, k_a_ee, k_c_ee, k_r_ee, k_d_ee, k_manip,
       k_r_force, velocity_max, approach_dist, detect_shell_rad);
};

void CfManager::init(
    const Eigen::Vector3d goal_pos, const double delta_t,
    const std::vector<Obstacle> &obstacles, const std::vector<double> &k_a_ee,
    const std::vector<double> &k_c_ee, const std::vector<double> &k_r_ee,
    const std::vector<double> &k_d_ee, const std::vector<double> &k_manip,
    const std::vector<double> &k_r_force, const double velocity_max,
    const double approach_dist, const double detect_shell_rad,
    const size_t max_prediction_steps, const size_t prediction_freq_multiple,
    const double agent_mass, const double radius) {
  assert((k_a_ee.size() == k_c_ee.size()) && (k_c_ee.size() == k_r_ee.size()) &&
         (k_c_ee.size() == k_manip.size()));

  joinPredictionThreads();
  ee_agents_.clear();
  goal_pos_ = goal_pos;
  k_a_ee_ = k_a_ee;
  k_c_ee_ = k_c_ee;
  k_r_ee_ = k_r_ee;
  k_d_ee_ = k_d_ee;
  k_manip_ = k_manip;
  k_r_force_ = k_r_force;
  run_prediction_ = false;
  int obstacle_size = obstacles.size();
  approach_dist_ = approach_dist;

  real_ee_agent_ =
      RealCfAgent(0, init_pos_, goal_pos, detect_shell_rad, agent_mass, radius,
                  velocity_max, approach_dist, obstacle_size);

  ee_agents_.push_back(std::make_unique<HadHeuristicCfAgent>(
      ee_agents_.size() + 1, init_pos_, goal_pos, detect_shell_rad, agent_mass,
      radius, velocity_max, approach_dist, obstacle_size, obstacles));

  if (k_a_ee.size() > ee_agents_.size()) {
    ee_agents_.push_back(std::make_unique<GoalHeuristicCfAgent>(
        ee_agents_.size() + 1, init_pos_, goal_pos, detect_shell_rad,
        agent_mass, radius, velocity_max, approach_dist, obstacle_size,
        obstacles));
  }
  if (k_a_ee.size() > ee_agents_.size()) {
    ee_agents_.push_back(std::make_unique<ObstacleHeuristicCfAgent>(
        ee_agents_.size() + 1, init_pos_, goal_pos, detect_shell_rad,
        agent_mass, radius, velocity_max, approach_dist, obstacle_size,
        obstacles));
  }
  if (k_a_ee.size() > ee_agents_.size()) {
    ee_agents_.push_back(std::make_unique<GoalObstacleHeuristicCfAgent>(
        ee_agents_.size() + 1, init_pos_, goal_pos, detect_shell_rad,
        agent_mass, radius, velocity_max, approach_dist, obstacle_size,
        obstacles));
  }
  if (k_a_ee.size() > ee_agents_.size()) {
    ee_agents_.push_back(std::make_unique<VelHeuristicCfAgent>(
        ee_agents_.size() + 1, init_pos_, goal_pos, detect_shell_rad,
        agent_mass, radius, velocity_max, approach_dist, obstacle_size,
        obstacles));
  }

  for (size_t i = ee_agents_.size(); i < k_a_ee.size(); ++i) {
    ee_agents_.push_back(std::make_unique<RandomCfAgent>(
        ee_agents_.size() + 1, init_pos_, goal_pos, detect_shell_rad,
        agent_mass, radius, velocity_max, approach_dist, obstacle_size,
        obstacles));
  }
  Vector3d zeros{0.0, 0.0, 0.0};
  for (size_t i = 0; i < k_r_force.size(); ++i) {
    force_agents_.push_back(std::make_unique<GoalHeuristicCfAgent>(
        force_agents_.size() + ee_agents_.size() + 1, init_pos_, goal_pos,
        detect_shell_rad, agent_mass, radius, velocity_max, approach_dist,
        obstacle_size, obstacles));
  }

  int dim_size = 60;
  manip_map_ = std::vector<Vector3d>(dim_size * dim_size * dim_size);
  // std::ifstream fin("manipulability_dim_60_res_5_Vector", std::ios::in |
  // std::ios::binary); fin.read((char *)(&manip_map_[0]), dim_size * dim_size
  // * dim_size * sizeof(Vector3d)); fin.close();
  for (size_t i = 0; i < ee_agents_.size(); i++) {
    prediction_threads_.push_back(std::thread(
        &CfAgent::cfPrediction, std::ref(*ee_agents_[i]), std::ref(manip_map_),
        k_a_ee_[i], k_c_ee_[i], k_r_ee_[i], k_d_ee_[i], k_manip_[i],
        prediction_freq_multiple * delta_t, max_prediction_steps));
  }
}

void CfManager::stopPrediction() {
  for (auto &ee_agent : ee_agents_) {
    ee_agent->stopPrediction();
  }
  // Wait for all agents to finish their prediction loop
  bool agent_running = true;
  while (agent_running) {
    agent_running = false;
    for (auto &agent : ee_agents_) {
      if (agent->getRunningStatus()) {
        agent_running = true;
      }
    }
  }
};

void CfManager::joinPredictionThreads() {
  // Stop predictions
  for (auto &ee_agent : ee_agents_) {
    ee_agent->stopPrediction();
    ee_agent->shutdownAgent();
  }
  // Join and delete prediction threads
  for (int i = prediction_threads_.size() - 1; i >= 0; i--) {
    if (prediction_threads_.at(i).joinable()) {
      // It is only possible to get the thread handle before it is joined
      pthread_t thread_handle = prediction_threads_.at(i).native_handle();
      // Merges thread back into program
      prediction_threads_.at(i).join();
      // This is needed to really free OS memory
      pthread_cancel(thread_handle);
      // Calling the destructor apparently only works if the thread was
      // not joined before and the it would terminate all threads? See
      // http://www.bo-yang.net/2017/11/19/cpp-kill-detached-thread
      // prediction_threads_.at(i).~thread();
      // Delete thread from vector
      prediction_threads_.pop_back();
    } else {
      std::cout << "not joinable" << std::endl; /* code */
    }
  }
}

std::vector<Vector3d> CfManager::getLinkForce(
    const std::vector<Eigen::Vector3d> &link_positions,
    const std::vector<Obstacle> &obstacles) {
  assert((force_agents_.size() == k_r_force_.size()) &&
         (k_r_force_.size() == link_positions.size()));

  std::vector<Vector3d> forces;
  for (size_t i = 0; i < force_agents_.size(); ++i) {
    force_agents_[i]->setPosition(link_positions[i]);
    Vector3d force = force_agents_[i]->bodyForce(obstacles, k_r_force_[i]);
    forces.push_back(force);
  }
  return forces;
};

std::vector<std::vector<Eigen::Vector3d>> CfManager::getPredictedPaths() {
  std::vector<std::vector<Eigen::Vector3d>> predicted_paths;
  for (size_t i = 0; i < ee_agents_.size(); i++) {
    predicted_paths.push_back(ee_agents_[i]->getPath());
  }
  return predicted_paths;
}

std::vector<double> CfManager::getPredictedPathLengths() {
  std::vector<double> path_lengths;
  for (size_t i = 0; i < ee_agents_.size(); i++) {
    path_lengths.push_back(ee_agents_[i]->getPathLength());
  }
  return path_lengths;
}

std::vector<double> CfManager::getPredictionTimes() {
  std::vector<double> pred_time;
  for (size_t i = 0; i < ee_agents_.size(); i++) {
    pred_time.push_back(ee_agents_[i]->getPredictionTime());
  }
  return pred_time;
}

std::vector<bool> CfManager::getAgentSuccess() {
  std::vector<bool> success;
  for (size_t i = 0; i < ee_agents_.size(); i++) {
    success.push_back(ee_agents_[i]->getReachedGoal());
  }
  return success;
}

void CfManager::setRealEEAgentPosition(const Eigen::Vector3d &position) {
  real_ee_agent_.setPosition(position);
};

void CfManager::setEEAgentPositions(const Eigen::Vector3d &position) {
  for (auto &agent : ee_agents_) {
    agent->setPosition(position);
  }
};

void CfManager::setInitialPosition(const Eigen::Vector3d &position) {
  init_pos_ = position;
  setInitialEEPositions(position);
};

void CfManager::setInitialEEPositions(const Eigen::Vector3d &position) {
  real_ee_agent_.setInitalPosition(position);
  for (auto &agent : ee_agents_) {
    agent->setInitalPosition(position);
  }
};

void CfManager::setEEAgentPosAndVels(const Eigen::Vector3d &position,
                                     const Eigen::Vector3d &velocity) {
  for (auto &agent : ee_agents_) {
    agent->setPosition(position);
    agent->setVelocity(velocity);
  }
};

void CfManager::resetEEAgents(const Eigen::Vector3d &position,
                              const Eigen::Vector3d &velocity,
                              const std::vector<Obstacle> &obstacles) {
  for (auto &agent : ee_agents_) {
    agent->setPosition(position);
    agent->setVelocity(velocity);
    agent->setObstacles(obstacles, real_ee_agent_.getKnownObstacles());
    agent->resetMinObsDist();
  }
};

void CfManager::moveRealEEAgent(const vector<Obstacle> &obstacles,
                                const double delta_t, const int steps,
                                const int agent_id) {
  real_ee_agent_.cfPlanner(
      manip_map_, obstacles, *best_agent_, k_a_ee_[agent_id], k_c_ee_[agent_id],
      k_r_ee_[agent_id], k_d_ee_[agent_id], k_manip_[agent_id], delta_t, steps);
};

void CfManager::moveAgent(const vector<Obstacle> &obstacles,
                          const double delta_t, const int steps, const int id) {
  while (run_prediction_ && ee_agents_[id]->getDistFromGoal() > 0.05) {
    ee_agents_[id]->cfPlanner(manip_map_, obstacles, k_a_ee_[id], k_c_ee_[id],
                              k_r_ee_[id], k_d_ee_[id], k_manip_[id], delta_t,
                              steps);
  }
};

void CfManager::moveAgents(const vector<Obstacle> &obstacles,
                           const double delta_t, const int steps) {
  for (size_t i = 0; i < ee_agents_.size(); ++i) {
    ee_agents_[i]->cfPlanner(manip_map_, obstacles, k_a_ee_[i], k_c_ee_[i],
                             k_r_ee_[i], k_d_ee_[i], k_manip_[i], delta_t,
                             steps);
  }
};

void CfManager::moveAgentsPar(const vector<Obstacle> &obstacles,
                              const double delta_t, const int steps) {
#pragma omp parallel for
  for (auto it = ee_agents_.begin(); it < ee_agents_.end(); it++) {
    int idx = it - ee_agents_.begin();
    (*it)->cfPlanner(manip_map_, obstacles, k_a_ee_[idx], k_c_ee_[idx],
                     k_r_ee_[idx], k_d_ee_[idx], k_manip_[idx], delta_t, steps);
  }
};

int CfManager::evaluateAgents(const vector<Obstacle> &obstacles,
                              const double k_goal_dist, const double k_path_len,
                              const double k_safe_dist,
                              const double k_workspace,
                              const Eigen::Matrix<double, 6, 1> des_ws_limits) {
  assert(ee_agents_.size() != 0);
  std::vector<double> costs;
  for (auto &&agent : ee_agents_) {
    double cost = 0;
    for (auto &&position : agent->getPath()) {
      if (position.x() > des_ws_limits(0)) {
        cost += std::pow(
            std::abs(position.x() - des_ws_limits(0)) * k_workspace, 2);
      } else if (position.x() < des_ws_limits(1)) {
        cost += std::pow(
            std::abs(position.x() - des_ws_limits(1)) * k_workspace, 2);
      }
      if (position.y() > des_ws_limits(2)) {
        cost += std::pow(
            std::abs(position.y() - des_ws_limits(2)) * k_workspace, 2);
      } else if (position.y() < des_ws_limits(3)) {
        cost += std::pow(
            std::abs(position.y() - des_ws_limits(3)) * k_workspace, 2);
      }
      if (position.z() > des_ws_limits(4)) {
        cost += std::pow(
            std::abs(position.z() - des_ws_limits(4)) * k_workspace, 2);
      } else if (position.z() < des_ws_limits(5)) {
        cost += std::pow(
            std::abs(position.z() - des_ws_limits(5)) * k_workspace, 2);
      }
    }
    double goal_dist = agent->getDistFromGoal();
    if (goal_dist > approach_dist_) {
      cost += goal_dist * k_goal_dist;
    }
    cost += agent->getPathLength() * k_path_len;
    cost += k_safe_dist / agent->getMinObsDist();
    if (agent->getMinObsDist() < 2e-5) {
      cost += 10000.0;
    }
    costs.push_back(cost);
  }
  int min_cost_idx = 0;
  double min_cost = std::numeric_limits<double>::max();
  for (size_t idx = 0; idx < ee_agents_.size(); ++idx) {
    if (costs[idx] < min_cost) {
      min_cost = costs[idx];
      min_cost_idx = idx;
    }
  }
  if (best_agent_) {
    if (costs[min_cost_idx] < 0.9 * costs[best_agent_->getAgentID() - 1]) {
      best_agent_ = ee_agents_[min_cost_idx]->makeCopy();
    }
    else{
      min_cost_idx = best_agent_->getAgentID() - 1;
    }
  } else {
    best_agent_ = ee_agents_[min_cost_idx]->makeCopy();
  }

  return min_cost_idx;
};

}  // namespace cfplanner
}  // namespace ghostplanner
