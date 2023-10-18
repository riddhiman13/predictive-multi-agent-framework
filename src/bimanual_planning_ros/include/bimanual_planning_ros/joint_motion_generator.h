// Copyright (c) 2020 Lehrstuhl für Robotik und Sysstemintelligenz, TU München
#pragma once

#include <mutex>

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 14, 1> Vector14d;

class JointMotionGenerator {
 public:
  static void generateC1Trajectory(std::mutex& m, Vector14d& q_d,
    Vector14d& qD_d, const double& v, const Vector14d& q,
    const Vector14d& goal);
};
