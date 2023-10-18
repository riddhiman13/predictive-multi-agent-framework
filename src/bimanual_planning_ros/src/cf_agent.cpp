#include "bimanual_planning_ros/cf_agent.h"

#include <cassert>
#include <random>

#include "bimanual_planning_ros/helper_functions.h"
#include "bimanual_planning_ros/obstacle.h"

namespace ghostplanner {
namespace cfplanner {
Eigen::Vector3d CfAgent::getFirstPosition() const {
  assert(pos_.size() != 0);
  return pos_[0];
}

Eigen::Vector3d CfAgent::getPosition(int id) const {
  assert(pos_.size() >= id);
  return pos_[id];
}

Eigen::Vector3d CfAgent::getLatestPosition() const {
  assert(pos_.size() != 0);
  return pos_.back();
}

double CfAgent::getPathLength() const {
  double path_length = 0;
  for (size_t i = 0; i < pos_.size() - 1; i++) {
    path_length += (pos_.at(i + 1) - pos_.at(i)).norm();
  }
  return path_length;
}

void CfAgent::setInitalPosition(Eigen::Vector3d position) {
  init_pos_ = position;
  setPosition(position);
}

void CfAgent::setPosition(Eigen::Vector3d position) {
  pos_.clear();
  pos_.push_back(position);
}

void RealCfAgent::setPosition(Eigen::Vector3d position) {
  pos_.push_back(position);
}

void CfAgent::reset() {
  Eigen::Vector3d one_step_pos{getFirstPosition()};
  pos_.clear();
  pos_.push_back(one_step_pos);
}

void CfAgent::setVelocity(const Eigen::Vector3d &velocity) {
  double velocity_norm = velocity.norm();
  if (velocity_norm > vel_max_) {
    vel_ = (vel_max_ / velocity_norm) * velocity;
  } else {
    vel_ = velocity;
  }
}

void CfAgent::setObstacles(const std::vector<Obstacle> &obstacles,
                           const std::vector<bool> real_known_obstacles) {
  for (int i = 0; i < obstacles.size(); ++i) {
    obstacles_.at(i).setPosition(obstacles.at(i).getPosition());
    obstacles_.at(i).setVelocity(obstacles.at(i).getVelocity());
    known_obstacles_.at(i) = real_known_obstacles.at(i);
  }
}

void CfAgent::circForce(const std::vector<Obstacle> &obstacles,
                        const double k_circ) {
  Eigen::Vector3d goal_vec{g_pos_ - getLatestPosition()};
  for (int i = 0; i < obstacles.size() - 1; i++) {
    Eigen::Vector3d robot_obstacle_vec{obstacles.at(i).getPosition() -
                                       getLatestPosition()};
    Eigen::Vector3d rel_vel{vel_ - obstacles.at(i).getVelocity()};
    if (robot_obstacle_vec.normalized().dot(goal_vec.normalized()) < -0.01 &&
        robot_obstacle_vec.dot(rel_vel) < -0.01) {
      continue;
    }
    double dist_obs{robot_obstacle_vec.norm() -
                    (rad_ + obstacles.at(i).getRadius())};
    dist_obs = std::max(dist_obs, 1e-5);
    if (dist_obs < min_obs_dist_) {
      min_obs_dist_ = dist_obs;
    }
    Eigen::Vector3d curr_force{0.0, 0.0, 0.0};
    Eigen::Vector3d current;
    if (dist_obs < detect_shell_rad_) {
      if (!known_obstacles_.at(i)) {
        field_rotation_vecs_.at(i) = calculateRotationVector(
            getLatestPosition(), getGoalPosition(), obstacles, i);
        known_obstacles_.at(i) = true;
      }
      double vel_norm = rel_vel.norm();
      if (vel_norm != 0) {
        Eigen::Vector3d normalized_vel = rel_vel / vel_norm;
        current = currentVector(getLatestPosition(), rel_vel, getGoalPosition(),
                                obstacles, i, field_rotation_vecs_);
        curr_force = (k_circ / pow(dist_obs, 2)) *
                     normalized_vel.cross(current.cross(normalized_vel));
      }
    }
    force_ += curr_force;
  }
}

void RealCfAgent::circForce(const std::vector<Obstacle> &obstacles,
                            const double k_circ, const CfAgent &agent) {
  Eigen::Vector3d goal_vec{g_pos_ - getLatestPosition()};
  for (int i = 0; i < obstacles.size() - 1; i++) {
    Eigen::Vector3d robot_obstacle_vec{obstacles.at(i).getPosition() -
                                       getLatestPosition()};
    Eigen::Vector3d rel_vel{vel_ - obstacles.at(i).getVelocity()};
    if (robot_obstacle_vec.normalized().dot(goal_vec.normalized()) < -0.01 &&
        robot_obstacle_vec.dot(rel_vel) < -0.01) {
      continue;
    }
    double dist_obs{
        (getLatestPosition() - obstacles.at(i).getPosition()).norm() -
        (rad_ + obstacles.at(i).getRadius())};
    dist_obs = std::max(dist_obs, 1e-5);
    Eigen::Vector3d curr_force{0.0, 0.0, 0.0};
    Eigen::Vector3d current;
    if (dist_obs < detect_shell_rad_) {
      if (!known_obstacles_.at(i)) {
        field_rotation_vecs_.at(i) = calculateRotationVector(
            getLatestPosition(), getGoalPosition(), obstacles, i, agent);
        known_obstacles_.at(i) = true;
      }
      double vel_norm = rel_vel.norm();
      if (vel_norm != 0) {
        Eigen::Vector3d normalized_vel = rel_vel / vel_norm;
        current = currentVector(getLatestPosition(), rel_vel, getGoalPosition(),
                                obstacles, i, field_rotation_vecs_, agent);
        curr_force = (k_circ / pow(dist_obs, 2)) *
                     normalized_vel.cross(current.cross(normalized_vel));
      }
    }
    force_ += curr_force;
  }
}

double CfAgent::evalObstacleDistance(
    const std::vector<Obstacle> &obstacles) const {
  double min_dist_obs = detect_shell_rad_;
  for (const Obstacle &obstacle : obstacles) {
    double dist_obs{(getLatestPosition() - obstacle.getPosition()).norm() -
                    (rad_ + obstacle.getRadius())};
    if (min_dist_obs > dist_obs) {
      min_dist_obs = dist_obs;
    }
  }
  return min_dist_obs;
}

void CfAgent::repelForce(const std::vector<Obstacle> &obstacles,
                         const double k_repel) {
  Eigen::Vector3d goal_vec{g_pos_ - getLatestPosition()};

  Eigen::Vector3d total_repel_force{0.0, 0.0, 0.0};
  Eigen::Vector3d robot_obstacle_vec{obstacles.back().getPosition() -
                                     getLatestPosition()};
  Eigen::Vector3d rel_vel{vel_ - obstacles.back().getVelocity()};
  Eigen::Vector3d dist_vec = -robot_obstacle_vec;
  double dist_obs{dist_vec.norm() - (rad_ + obstacles.back().getRadius())};
  dist_obs = std::max(dist_obs, 1e-5);
  Eigen::Vector3d repel_force{0.0, 0.0, 0.0};
  if (dist_obs < detect_shell_rad_) {
    Eigen::Vector3d obs_to_robot =
        getLatestPosition() - obstacles.back().getPosition();
    obs_to_robot.normalize();
    repel_force = k_repel * obs_to_robot *
                  (1.0 / dist_obs - 1.0 / detect_shell_rad_) /
                  (dist_obs * dist_obs);
  }
    total_repel_force += repel_force;
    force_ += total_repel_force;
}

void CfAgent::attractorForce(const double k_attr, const double k_damp,
                             const double k_goal_scale) {
  if (k_attr == 0.0) {
    return;
  }
  Eigen::Vector3d goal_vec{g_pos_ - getLatestPosition()};
  Eigen::Vector3d vel_des = k_attr / k_damp * goal_vec;
  double scale_lim = std::min(1.0, vel_max_ / vel_des.norm());
  vel_des *= scale_lim;
  force_ += k_goal_scale * k_damp * (vel_des - vel_);
}

double CfAgent::attractorForceScaling(const std::vector<Obstacle> &obstacles) {
  double w1;
  double w2;
  int id_closest_obstacle;
  bool no_close_obs = true;
  double closest_obs_dist = detect_shell_rad_;
  for (int i = 0; i < obstacles.size() - 1; i++) {
    double dist_obs{
        (getLatestPosition() - obstacles.at(i).getPosition()).norm() -
        (rad_ + obstacles.at(i).getRadius())};
    dist_obs = std::max(dist_obs, 1e-5);
    if (dist_obs < closest_obs_dist) {
      no_close_obs = false;
      closest_obs_dist = dist_obs;
      id_closest_obstacle = i;
    }
  }
  if (no_close_obs) {
    return 1;
  }
  Eigen::Vector3d goal_vec{g_pos_ - getLatestPosition()};
  if (goal_vec.dot(vel_) <= 0.0 && vel_.norm() < vel_max_ - 0.1 * vel_max_ &&
      goal_vec.norm() > 0.15) {
    return 0.0;
  }
  w1 = 1 - std::exp(-std::sqrt(closest_obs_dist) / detect_shell_rad_);
  Eigen::Vector3d robot_obstacle_vec{
      obstacles.at(id_closest_obstacle).getPosition() - getLatestPosition()};
  w2 = 1 - (goal_vec.dot(robot_obstacle_vec) /
            (goal_vec.norm() * robot_obstacle_vec.norm()));
  w2 = w2 * w2;
  return w1 * w2;
}

Eigen::Vector3d CfAgent::bodyForce(const std::vector<Obstacle> &obstacles,
                                   const double k_repel) {
  resetForce();
  repelForce(obstacles, k_repel);
  return force_;
}

void CfAgent::manipulabilityForce(const std::vector<Eigen::Vector3d> &manip_map,
                                  double k_manip, double offset,
                                  double resolution) {
  int dim_size = 60;
  assert(manip_map.size() == dim_size * dim_size * dim_size);
  Eigen::Vector3d pos_cm =
      (100 * getLatestPosition()) + Eigen::Vector3d{offset, offset, offset};
  int idx_x = (int)(pos_cm.x() / resolution);
  int idx_y = (int)(pos_cm.y() / resolution);
  int idx_z = (int)(pos_cm.z() / resolution);
  if ((idx_x >= 0) && (idx_x < dim_size) && (idx_y >= 0) &&
      (idx_y < dim_size) && (idx_z >= 0) && (idx_z < dim_size)) {
    int idx = idx_x + dim_size * idx_y + dim_size * dim_size * idx_z;
    force_ += k_manip * manip_map[idx];
  }
}

void CfAgent::updatePositionAndVelocity(const double delta_t) {
  Eigen::Vector3d robot_acc = force_ / mass_;
  double acc_norm = robot_acc.norm();
  if (acc_norm > 13.0) {
    robot_acc *= 13.0 / acc_norm;
  }
  Eigen::Vector3d new_pos = getLatestPosition() +
                            0.5 * robot_acc * delta_t * delta_t +
                            vel_ * delta_t;
  vel_ += robot_acc * delta_t;
  double vel_norm = vel_.norm();
  if (vel_norm > vel_max_) {
    vel_ *= vel_max_ / vel_norm;
  }
  pos_.push_back(new_pos);
}

void CfAgent::predictObstacles(const double delta_t) {
  for (auto &&obstacle : obstacles_) {
    Eigen::Vector3d new_pos =
        obstacle.getPosition() + obstacle.getVelocity() * delta_t;
    obstacle.setPosition(new_pos);
  }
}

void CfAgent::cfPlanner(const std::vector<Eigen::Vector3d> &manip_map,
                        const std::vector<Obstacle> &obstacles,
                        const double k_attr, const double k_circ,
                        const double k_repel, const double k_damp,
                        const double k_manip, const double delta_t,
                        const int steps) {
  for (size_t i = 0; i < steps; ++i) {
    resetForce();
    double k_goal_scale = 1.0;
    if (!(getDistFromGoal() < approach_dist_ ||
          (vel_.norm() < 0.5 * vel_max_ &&
           (getLatestPosition() - init_pos_).norm() < 0.2))) {
      circForce(obstacles, k_circ);
      if (force_.norm() > 1e-5) {
        k_goal_scale = attractorForceScaling(obstacles);
      }
    }
    repelForce(obstacles, k_repel);
    attractorForce(k_attr, k_damp, k_goal_scale);
    // manipulabilityForce(manip_map, k_manip, 150.0, 5.0);
    updatePositionAndVelocity(delta_t);
  }
}

void CfAgent::cfPrediction(const std::vector<Eigen::Vector3d> &manip_map,
                           const double k_attr, const double k_circ,
                           const double k_repel, const double k_damp,
                           const double k_manip, const double delta_t,
                           const size_t max_prediction_steps) {
  while (!finished_) {
    std::chrono::steady_clock::time_point begin_prediction =
        std::chrono::steady_clock::now();
    while (run_prediction_ && getDistFromGoal() > 0.1 &&
           pos_.size() < max_prediction_steps) {
      running_ = true;
      resetForce();
      double k_goal_scale = 1.0;
      if (!(getDistFromGoal() < approach_dist_ ||
            (vel_.norm() < 0.5 * vel_max_ &&
             (getLatestPosition() - init_pos_).norm() < 0.2))) {
        circForce(obstacles_, k_circ);
        if (force_.norm() > 1e-5) {
          k_goal_scale = attractorForceScaling(obstacles_);
        }
      }
      repelForce(obstacles_, k_repel);
      attractorForce(k_attr, k_damp, k_goal_scale);
      // manipulabilityForce(manip_map, k_manip, 150.0, 5.0);
      updatePositionAndVelocity(delta_t);
      predictObstacles(delta_t);
    }
    auto end_prediction = std::chrono::steady_clock::now();
    if (running_) {
      prediction_time_ = (end_prediction - begin_prediction).count();
      if (getDistFromGoal() < 0.100001) {
        reached_goal_ = true;
      } else {
        reached_goal_ = false;
      }
    }

    running_ = false;
  }
}

void RealCfAgent::cfPlanner(const std::vector<Eigen::Vector3d> &manip_map,
                            const std::vector<Obstacle> &obstacles,
                            const CfAgent &agent, const double k_attr,
                            const double k_circ, const double k_repel,
                            const double k_damp, const double k_manip,
                            const double delta_t, const int steps) {
  for (size_t i = 0; i < steps; ++i) {
    resetForce();
    double k_goal_scale = 1.0;
    if (!(getDistFromGoal() < approach_dist_ ||
          (vel_.norm() < 0.5 * vel_max_ &&
           (getLatestPosition() - init_pos_).norm() < 0.2))) {
      circForce(obstacles, k_circ, agent);
      if (force_.norm() > 1e-5) {
        k_goal_scale = attractorForceScaling(obstacles);
      }
    }
    repelForce(obstacles, k_repel);
    attractorForce(k_attr, k_damp, k_goal_scale);
    // repelForce(obstacles, k_repel);
    // manipulabilityForce(manip_map, k_manip, 150.0, 5.0);
    updatePositionAndVelocity(delta_t);
  }
}

Eigen::Vector3d RealCfAgent::currentVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
    const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
    const int obstacle_id,
    const std::vector<Eigen::Vector3d> field_rotation_vecs,
    const CfAgent &agent) const {
  Eigen::Vector3d current =
      agent.currentVector(agent_pos, agent_vel, goal_pos, obstacles,
                          obstacle_id, field_rotation_vecs);
  return current;
}

Eigen::Vector3d RealCfAgent::calculateRotationVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
    const std::vector<Obstacle> &obstacles, const int obstacle_id,
    const CfAgent &agent) const {
  Eigen::Vector3d rot_vec = agent.calculateRotationVector(
      agent_pos, goal_pos, obstacles, obstacle_id);
  return rot_vec;
}

Eigen::Vector3d GoalHeuristicCfAgent::currentVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
    const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
    const int obstacle_id,
    const std::vector<Eigen::Vector3d> field_rotation_vecs) const {
  Eigen::Vector3d goal_vec{goal_pos - agent_pos};
  Eigen::Vector3d cfagent_to_obs{obstacles[obstacle_id].getPosition() -
                                 agent_pos};
  cfagent_to_obs.normalize();
  Eigen::Vector3d current{goal_vec -
                          cfagent_to_obs * (cfagent_to_obs.dot(goal_vec))};
  if (current.norm() < 1e-10) {
    current << 0.0, 0.0, 1.0;
    // current = makeRandomVector();
  }
  current.normalize();
  return current;
}

Eigen::Vector3d GoalHeuristicCfAgent::calculateRotationVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
    const std::vector<Obstacle> &obstacles, const int obstacle_id) const {
  return {0.0, 0.0, 1.0};
}

Eigen::Vector3d ObstacleHeuristicCfAgent::currentVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
    const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
    const int obstacle_id,
    const std::vector<Eigen::Vector3d> field_rotation_vecs) const {
  Eigen::Vector3d cfagent_to_obs{obstacles[obstacle_id].getPosition() -
                                 agent_pos};
  cfagent_to_obs.normalize();
  Eigen::Vector3d current{
      cfagent_to_obs.cross(field_rotation_vecs.at(obstacle_id))};
  current.normalize();
  return current;
}

Eigen::Vector3d ObstacleHeuristicCfAgent::calculateRotationVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
    const std::vector<Obstacle> &obstacles, const int obstacle_id) const {
  if (obstacles.size() < 2) {
    return Eigen::Vector3d(0.0, 0.0, 1.0);
  }
  double min_dist_obs = 100.0;
  int closest_obstacle_it = 0;
  for (int i = 0; i < obstacles.size() - 1; i++) {
    if (i != obstacle_id) {
      double dist_obs{
          (obstacles[obstacle_id].getPosition() - obstacles[i].getPosition())
              .norm()};
      if (min_dist_obs > dist_obs) {
        min_dist_obs = dist_obs;
        closest_obstacle_it = i;
      }
    }
  }
  // Vector from active obstacle to the obstacle which is closest to the
  // active obstacle
  Eigen::Vector3d obstacle_vec = obstacles[closest_obstacle_it].getPosition() -
                                 obstacles[obstacle_id].getPosition();
  Eigen::Vector3d cfagent_to_obs{obstacles[obstacle_id].getPosition() -
                                 agent_pos};
  cfagent_to_obs.normalize();
  // Current vector is perpendicular to obstacle surface normal and shows in
  // opposite direction of obstacle_vec
  Eigen::Vector3d current{(cfagent_to_obs * obstacle_vec.dot(cfagent_to_obs)) -
                          obstacle_vec};
  Eigen::Vector3d rot_vec{current.cross(cfagent_to_obs)};
  rot_vec.normalize();
  return rot_vec;
}

Eigen::Vector3d GoalObstacleHeuristicCfAgent::currentVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
    const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
    const int obstacle_id,
    const std::vector<Eigen::Vector3d> field_rotation_vecs) const {
  Eigen::Vector3d cfagent_to_obs{obstacles[obstacle_id].getPosition() -
                                 agent_pos};
  cfagent_to_obs.normalize();
  Eigen::Vector3d current{
      cfagent_to_obs.cross(field_rotation_vecs.at(obstacle_id))};
  current.normalize();
  return current;
}

Eigen::Vector3d GoalObstacleHeuristicCfAgent::calculateRotationVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
    const std::vector<Obstacle> &obstacles, const int obstacle_id) const {
  double min_dist_obs = 100.0;
  int closest_obstacle_it = 0;
  for (int i = 0; i < obstacles.size() - 1; i++) {
    if (i != obstacle_id) {
      double dist_obs{
          (obstacles[obstacle_id].getPosition() - obstacles[i].getPosition())
              .norm()};
      if (min_dist_obs > dist_obs) {
        min_dist_obs = dist_obs;
        closest_obstacle_it = i;
      }
    }
  }
  // Vector from active obstacle to the obstacle which is closest to the
  // active obstacle
  Eigen::Vector3d obstacle_vec = obstacles[closest_obstacle_it].getPosition() -
                                 obstacles[obstacle_id].getPosition();
  Eigen::Vector3d cfagent_to_obs{obstacles[obstacle_id].getPosition() -
                                 agent_pos};
  cfagent_to_obs.normalize();
  // Current vector is perpendicular to obstacle surface normal and shows in
  // opposite direction of obstacle_vec
  Eigen::Vector3d obst_current{
      (cfagent_to_obs * obstacle_vec.dot(cfagent_to_obs)) - obstacle_vec};
  Eigen::Vector3d goal_vec{goal_pos - agent_pos};
  Eigen::Vector3d goal_current{goal_vec -
                               cfagent_to_obs * (cfagent_to_obs.dot(goal_vec))};
  Eigen::Vector3d current{goal_current.normalized() +
                          obst_current.normalized()};

  if (current.norm() < 1e-10) {
    current << 0.0, 0.0, 1.0;
    // current = makeRandomVector();
  }
  current.normalize();
  Eigen::Vector3d rot_vec{current.cross(cfagent_to_obs)};
  rot_vec.normalize();
  return rot_vec;
}

Eigen::Vector3d VelHeuristicCfAgent::currentVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
    const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
    const int obstacle_id,
    const std::vector<Eigen::Vector3d> field_rotation_vecs) const {
  Eigen::Vector3d normalized_vel = agent_vel.normalized();
  Eigen::Vector3d normalized_obs_to_agent{
      (obstacles[obstacle_id].getPosition() - agent_pos).normalized()};
  Eigen::Vector3d current{
      normalized_vel -
      (normalized_obs_to_agent * normalized_vel.dot(normalized_obs_to_agent))};
  if (current.norm() < 1e-10) {
    current << 0.0, 0.0, 1.0;
    // current = makeRandomVector();
  }
  current.normalize();
  return current;
}

Eigen::Vector3d VelHeuristicCfAgent::calculateRotationVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
    const std::vector<Obstacle> &obstacles, const int obstacle_id) const {
  return {0.0, 0.0, 1.0};
}

Eigen::Vector3d RandomCfAgent::currentVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
    const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
    const int obstacle_id,
    const std::vector<Eigen::Vector3d> field_rotation_vecs) const {
  Eigen::Vector3d cfagent_to_obs{obstacles[obstacle_id].getPosition() -
                                 agent_pos};
  cfagent_to_obs.normalize();
  Eigen::Vector3d current =
      cfagent_to_obs.cross(field_rotation_vecs.at(obstacle_id));
  current.normalize();
  return current;
}

Eigen::Vector3d RandomCfAgent::calculateRotationVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
    const std::vector<Obstacle> &obstacles, const int obstacle_id) const {
  Eigen::Vector3d goal_vec{goal_pos - agent_pos};
  goal_vec.normalize();
  Eigen::Vector3d rot_vec = goal_vec.cross(random_vecs_.at(obstacle_id));
  return rot_vec;
}

void RandomCfAgent::saveRandomVecToFile(const int num_obstacles) {
  std::ofstream outfile(
      "reactive_bimanual_manip/random_vectors/last_random_vectors.txt",
      std::ios_base::app);
  outfile << "if(getAgentID()== " << getAgentID() << "){" << std::endl;
  for (size_t i = 0; i < num_obstacles; i++) {
    Eigen::Vector3d random_vec = makeRandomVector();
    random_vec.normalize();
    random_vecs_.push_back(random_vec);
    outfile << "random_vec << " << random_vec.x() << ", " << random_vec.y()
            << ", " << random_vec.z() << ";" << std::endl;
    outfile << "random_vecs_.push_back(random_vec);" << std::endl;
  }
  outfile << "}" << std::endl << std::endl;
  outfile.close();
}

Eigen::Vector3d HadHeuristicCfAgent::currentVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d agent_vel,
    const Eigen::Vector3d goal_pos, const std::vector<Obstacle> &obstacles,
    const int obstacle_id,
    const std::vector<Eigen::Vector3d> field_rotation_vecs) const {
  Eigen::Vector3d cfagent_to_obs{obstacles[obstacle_id].getPosition() -
                                 agent_pos};
  cfagent_to_obs.normalize();
  Eigen::Vector3d current =
      cfagent_to_obs.cross(field_rotation_vecs.at(obstacle_id));
  current.normalize();
  return current;
}

Eigen::Vector3d HadHeuristicCfAgent::calculateRotationVector(
    const Eigen::Vector3d agent_pos, const Eigen::Vector3d goal_pos,
    const std::vector<Obstacle> &obstacles, const int obstacle_id) const {
  Eigen::Vector3d obs_pos = obstacles.at(obstacle_id).getPosition();
  Eigen::Vector3d goal_vec{goal_pos - agent_pos};
  Eigen::Vector3d rob_obs_vec{obs_pos - agent_pos};
  Eigen::Vector3d d =
      agent_pos +
      goal_vec * (rob_obs_vec.dot(goal_vec) / pow(goal_vec.norm(), 2)) -
      obs_pos;
  Eigen::Vector3d rot_vec = d.cross(goal_vec) / (d.cross(goal_vec)).norm();
  return rot_vec;
}

}  // namespace cfplanner
}  // namespace ghostplanner
